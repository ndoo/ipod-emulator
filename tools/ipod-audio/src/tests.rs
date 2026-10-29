use super::*;
use std::cell::Cell;

/// A device that is only a loop calling `fill` with fixed-size buffers.
struct Fake {
    player: Player,
    channels: usize,
    out: Vec<f32>,
}

impl Fake {
    fn new(player: Player, channels: usize) -> Self {
        Self {
            player,
            channels,
            out: Vec::new(),
        }
    }

    fn pull(&mut self, frames: usize) {
        let mut buf = vec![f32::NAN; frames * self.channels];
        self.player.fill(&mut buf, self.channels);
        self.out.extend(buf);
    }
}

/// `n` stereo frames numbered from `from`: left is the number, right is its negative.
fn ramp(from: usize, n: usize) -> Vec<f32> {
    (from..from + n)
        .flat_map(|i| [i as f32 / 100_000.0, -(i as f32) / 100_000.0])
        .collect()
}

fn cfg() -> Config {
    Config::default()
}

#[test]
fn the_callback_receives_the_samples_that_were_pushed() {
    let (mut feeder, player, stats) = pair(44100, cfg());
    let mut dev = Fake::new(player, 2);
    let pcm = ramp(0, 4410);
    // Pushed as the machine pushes them: a slice at a time.
    for chunk in pcm.chunks(2 * 300) {
        feeder.push(44100, chunk);
    }
    // Pulled as a device pulls: 512-frame buffers.
    for _ in 0..10 {
        dev.pull(512);
    }
    assert_eq!(&dev.out[..pcm.len()], &pcm[..], "exact, in order");
    assert!(dev.out[pcm.len()..].iter().all(|&s| s == 0.0));
    let s = stats.snapshot();
    assert_eq!(s.pushed_frames, 4410);
    assert_eq!(s.played_frames, 4410);
    assert_eq!(s.underruns, 1, "the tail ran dry once");
    assert_eq!(s.dropped_frames, 0);
}

#[test]
fn nothing_plays_until_the_preroll_is_queued() {
    let (mut feeder, player, stats) = pair(48000, cfg());
    let mut dev = Fake::new(player, 2);
    // 50 ms at 48 kHz is 2400 frames. 2000 is short of it.
    feeder.push(48000, &ramp(0, 2000));
    dev.pull(256);
    assert!(dev.out.iter().all(|&s| s == 0.0));
    assert_eq!(stats.snapshot().played_frames, 0);
    assert_eq!(
        stats.snapshot().silence_frames,
        0,
        "not started, so not an underrun"
    );
    feeder.push(48000, &ramp(2000, 1000));
    dev.out.clear();
    dev.pull(256);
    assert_eq!(&dev.out[..], &ramp(0, 256)[..], "starts at the first frame");
}

#[test]
fn an_underrun_is_silence_and_resumes_without_losing_or_repeating_a_sample() {
    let (mut feeder, player, stats) = pair(44100, cfg());
    let mut dev = Fake::new(player, 2);
    feeder.push(44100, &ramp(0, 3000));
    dev.pull(2000);
    assert_eq!(&dev.out[..], &ramp(0, 2000)[..]);
    dev.out.clear();
    // 1000 frames left; ask for 1500: the last 500 are silence.
    dev.pull(1500);
    assert_eq!(&dev.out[..2000], &ramp(2000, 1000)[..]);
    assert!(
        dev.out[2000..].iter().all(|&s| s == 0.0),
        "silence, not a repeat"
    );
    assert_eq!(stats.snapshot().underruns, 1);
    assert_eq!(stats.snapshot().silence_frames, 500);
    // A little more arrives, short of the preroll: still silent.
    feeder.push(44100, &ramp(3000, 1000));
    dev.out.clear();
    dev.pull(256);
    assert!(dev.out.iter().all(|&s| s == 0.0));
    // Enough arrives: playback resumes at the next frame in order.
    feeder.push(44100, &ramp(4000, 2000));
    dev.out.clear();
    dev.pull(256);
    assert_eq!(&dev.out[..], &ramp(3000, 256)[..]);
}

#[test]
fn a_ring_that_is_full_drops_the_newest_and_counts_them() {
    let (mut feeder, player, stats) = pair(44100, cfg());
    let mut dev = Fake::new(player, 2);
    let capacity = 22050; // 500 ms
    feeder.push(44100, &ramp(0, capacity + 5000));
    let s = stats.snapshot();
    assert_eq!((s.pushed_frames, s.dropped_frames), (capacity as u64, 5000));
    assert_eq!(feeder.queued_frames(), capacity);
    dev.pull(1000);
    assert_eq!(&dev.out[..], &ramp(0, 1000)[..], "the oldest are what play");
}

#[test]
fn channel_counts_other_than_two_are_mapped() {
    let (mut feeder, player, _) = pair(44100, cfg());
    let mut mono = Fake::new(player, 1);
    feeder.push(44100, &[0.5, 0.25].repeat(3000));
    mono.pull(4);
    assert_eq!(mono.out, vec![0.375; 4]);

    let (mut feeder, player, _) = pair(44100, cfg());
    let mut six = Fake::new(player, 6);
    feeder.push(44100, &[0.5, 0.25].repeat(3000));
    six.pull(2);
    assert_eq!(six.out, [0.5, 0.25, 0.0, 0.0, 0.0, 0.0].repeat(2));
}

#[test]
fn a_rate_change_in_the_stream_is_resampled_to_the_device_rate() {
    let (mut feeder, player, stats) = pair(48000, cfg());
    let mut dev = Fake::new(player, 2);
    // One second at 24 kHz is one second at 48 kHz: 48000 frames, to within a frame.
    let dc = vec![0.25f32; 2 * 24000];
    feeder.push(24000, &dc[..2 * 12000]);
    feeder.push(24000, &dc[2 * 12000..]);
    let queued = feeder.queued_frames();
    assert!((queued as i64 - 22000).abs() <= 2000, "{queued}");
    // The ring only holds 500 ms, so what did not fit was dropped and counted.
    assert_eq!(
        stats.snapshot().pushed_frames + stats.snapshot().dropped_frames,
        47998,
        "24000 input frames give 47998 outputs: the stream ends on its last input frame"
    );
    dev.pull(1000);
    assert!(
        dev.out.iter().all(|&s| (s - 0.25).abs() < 1e-6),
        "DC survives"
    );
}

#[test]
fn throttle_returns_at_once_while_the_device_is_not_far_behind() {
    let (mut feeder, _player, _) = pair(44100, cfg());
    feeder.push(44100, &ramp(0, 5000));
    let slept = Cell::new(0u32);
    feeder.throttle_with(|| false, |_| slept.set(slept.get() + 1));
    assert_eq!(slept.get(), 0);
}

#[test]
fn throttle_holds_the_machine_until_the_device_has_caught_up() {
    let (mut feeder, player, stats) = pair(44100, cfg());
    let mut dev = Fake::new(player, 2);
    // Prime the device with a little audio so it has started.
    feeder.push(44100, &ramp(0, 3000));
    dev.pull(1000);
    // The machine runs ahead: 400 ms queued, above the 250 ms high mark.
    feeder.push(44100, &ramp(3000, 17640 - 2000));
    assert!(feeder.queued_frames() > 11025);
    let mut slept_ms = 0u32;
    feeder.throttle_with(
        || false,
        |d| {
            // The device plays for as long as the thread slept.
            slept_ms += d.as_millis() as u32;
            dev.pull((44100 * d.as_millis() as usize) / 1000);
        },
    );
    assert!(feeder.queued_frames() <= 6615, "back to the 150 ms mark");
    assert!(
        (240..=260).contains(&slept_ms),
        "{slept_ms} ms for 250 ms of audio (400 ms down to 150 ms)"
    );
    assert_eq!(stats.snapshot().dropped_frames, 0);
    assert_eq!(stats.snapshot().underruns, 0);
}

#[test]
fn throttle_gives_up_on_a_device_that_has_stopped_and_pushes_then_drop() {
    let (mut feeder, _player, stats) = pair(44100, cfg());
    feeder.push(44100, &ramp(0, 20000));
    let mut slept_ms = 0u32;
    feeder.throttle_with(|| false, |d| slept_ms += d.as_millis() as u32);
    assert_eq!(slept_ms, 250, "waited the stall time and no longer");
    // Stalled: the next call does not wait at all.
    feeder.throttle_with(|| false, |_| panic!("slept while stalled"));
    feeder.push(44100, &ramp(0, 10000));
    assert!(
        stats.snapshot().dropped_frames > 0,
        "the ring is full, so frames are dropped"
    );
}

#[test]
fn throttle_stops_waiting_when_asked_to_quit() {
    let (mut feeder, _player, _) = pair(44100, cfg());
    feeder.push(44100, &ramp(0, 20000));
    let n = Cell::new(0);
    feeder.throttle_with(|| n.get() >= 3, |_| n.set(n.get() + 1));
    assert_eq!(n.get(), 3);
}

#[test]
fn equal_rates_are_copied_exactly() {
    let mut r = Resampler::new(44100, 44100);
    let mut out = Vec::new();
    let pcm = ramp(0, 100);
    r.process(&pcm, &mut out);
    assert_eq!(out, pcm);
}

#[test]
fn a_resampled_stream_is_the_same_cut_anywhere() {
    let pcm: Vec<f32> = (0..2 * 5000)
        .map(|i| ((i / 2) as f32 * 0.01).sin() * if i % 2 == 0 { 1.0 } else { -0.5 })
        .collect();
    for (a, b) in [
        (44100, 48000),
        (48000, 44100),
        (8000, 48000),
        (22050, 44100),
    ] {
        let mut whole = Vec::new();
        Resampler::new(a, b).process(&pcm, &mut whole);
        for cut in [1usize, 2, 7, 333, 1000] {
            let mut r = Resampler::new(a, b);
            let mut parts = Vec::new();
            for c in pcm.chunks(cut * 2) {
                r.process(c, &mut parts);
            }
            assert_eq!(whole.len(), parts.len(), "{a}->{b} cut {cut}");
            for (x, y) in whole.iter().zip(&parts) {
                assert!((x - y).abs() < 1e-6, "{a}->{b} cut {cut}");
            }
        }
        let expect = 5000.0 * f64::from(b) / f64::from(a);
        assert!(
            (whole.len() as f64 / 2.0 - expect).abs() <= f64::from(b) / f64::from(a) + 1.0,
            "{a}->{b}: {} frames for {expect}",
            whole.len() / 2
        );
    }
}

#[test]
fn a_tone_keeps_its_frequency_across_a_rate_change() {
    let tone = |rate: u32, n: usize| -> Vec<f32> {
        (0..n)
            .flat_map(|i| {
                let v = (2.0 * std::f64::consts::PI * 1000.0 * i as f64 / f64::from(rate)).sin();
                [v as f32, v as f32]
            })
            .collect()
    };
    let mut out = Vec::new();
    Resampler::new(44100, 48000).process(&tone(44100, 44100), &mut out);
    let left: Vec<f32> = out.chunks(2).map(|f| f[0]).collect();
    let cross = (1..left.len())
        .filter(|&i| left[i - 1] < 0.0 && left[i] >= 0.0)
        .count();
    assert!(
        (cross as i64 - 1000).abs() <= 1,
        "{cross} cycles in one second"
    );
}

/// The whole chain except the host device: the codec model and I2S sink of a machine, the queue
/// the window drains after each slice, the ring, and a loop standing in for the callback.
#[test]
fn frames_stored_to_the_i2s_fifo_come_out_of_the_callback_scaled_by_the_codec() {
    use arm7tdmi::Bus as _;
    use ipod_machine::{map_hardware, EApp, Machine, Wm8758};

    let mut m = Machine::new(&EApp::none(), 0x1100_0000, 0x0100_0000);
    map_hardware(&mut m, false);
    m.mem.enable_audio_live(Wm8758::DEFAULT_MCLK_HZ);
    // The 44.1 kHz bring-up, then the 80% volume: DAC 255, OUT1 39, which is -18 dB.
    let put = |m: &mut Machine, reg: u8, val: u16| {
        m.mem
            .wm8758
            .transfer(0x82, [(reg << 1) | (val >> 8) as u8, val as u8, 0, 0]);
    };
    for (reg, val) in [
        (0x01, 0x02f),
        (0x03, 0x00f),
        (0x06, 0x001),
        (0x24, 0x017),
        (0x25, 0x021),
        (0x26, 0x161),
        (0x27, 0x026),
        (0x06, 0x145),
        (0x0b, 0x0ff),
        (0x0c, 0x1ff),
        (0x34, 0x0a7),
        (0x35, 0x1a7),
    ] {
        put(&mut m, reg, val);
    }
    m.mem.write32(0x7000_2800, 1 << 29);

    let (mut feeder, player, stats) = pair(48000, cfg());
    let mut dev = Fake::new(player, 2);
    let gain = 10f32.powf(-18.0 / 20.0);
    let mut sent = Vec::new();
    for n in 0..4410u32 {
        let (l, r) = ((n as i16).wrapping_mul(7), (n as i16).wrapping_mul(-3));
        m.mem.write32(
            0x7000_2840,
            u32::from(l as u16) | (u32::from(r as u16) << 16),
        );
        sent.extend([f32::from(l) / 32768.0 * gain, f32::from(r) / 32768.0 * gain]);
        // A slice's worth at a time, as the window does.
        if n % 300 == 299 {
            feeder.pump(&mut m.mem, || false);
        }
    }
    feeder.pump(&mut m.mem, || false);
    // 44.1 kHz in, 48 kHz out: compare against the same resampler run over the expected frames.
    let mut want = Vec::new();
    Resampler::new(44100, 48000).process(&sent, &mut want);
    for _ in 0..12 {
        dev.pull(512);
    }
    assert_eq!(&dev.out[..want.len()], &want[..]);
    assert!(dev.out[want.len()..].iter().all(|&s| s == 0.0));
    assert_eq!(stats.snapshot().pushed_frames as usize, want.len() / 2);
    assert_eq!(stats.snapshot().dropped_frames, 0);
}
