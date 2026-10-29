//! End to end for the WM8758 and I2S sink: a bare-metal guest (`snippets/audio-tone.s`) programs
//! the codec over I2C, starts a DMA ring of a 1 kHz tone and restarts it from the FIQ handler, and
//! the WAV that comes out has to be the stream the guest played, at the rate the guest's own PLL
//! writes decode to, for as long as the guest played it.
//!
//! The guest is real ARM code run on the interpreter; nothing here calls the sink directly except
//! the two tests that say so.

use arm7tdmi::Bus as _;
use ipod_machine::hw::audio::parse_wav;
use ipod_machine::snippet::parse_words;
use ipod_machine::{map_hardware, EApp, Machine, Wm8758};
use std::path::PathBuf;

const CODE: &str = include_str!("../snippets/audio-tone.words");

const PARAMS: u32 = 0x400;
const TABLE: u32 = 0x500;
const RING: u32 = 0x1000;
const PHYS: u32 = 0x1000_0000;
const RING_SEGS: u32 = 4;
const TONE_HZ: f64 = 1000.0;

struct Plan {
    rate: u32,
    regs: Vec<(u8, u16)>,
    seg_frames: u32,
    total_segs: u32,
}

fn gcd(a: u32, b: u32) -> u32 {
    if b == 0 {
        a
    } else {
        gcd(b, a % b)
    }
}

/// The register writes a driver makes to bring the codec up at `rate`: power, I2S format, the PLL
/// and divider for the rate, DAC mode, headphone volume. Values per `docs/hw/wm8758-i2s.md`.
fn bring_up(rate: u32) -> Vec<(u8, u16)> {
    let (pll, div, sr): ([u16; 4], u16, u16) = match rate {
        48000 => ([0x018, 0x00c, 0x093, 0x0e9], 2, 0),
        44100 => ([0x017, 0x021, 0x161, 0x026], 2, 0),
        32000 => ([0x018, 0x00c, 0x093, 0x0e9], 3, 1),
        24000 => ([0x018, 0x00c, 0x093, 0x0e9], 4, 2),
        22050 => ([0x017, 0x021, 0x161, 0x026], 4, 2),
        16000 => ([0x018, 0x00c, 0x093, 0x0e9], 5, 3),
        12000 => ([0x018, 0x00c, 0x093, 0x0e9], 6, 4),
        11025 => ([0x017, 0x021, 0x161, 0x026], 6, 4),
        8000 => ([0x018, 0x00c, 0x093, 0x0e9], 7, 5),
        _ => panic!("no PLL set for {rate}"),
    };
    let mut w = vec![
        (0x3d, 0x100),
        (0x31, 0x187),
        (0x34, 0x140),
        (0x35, 0x140),
        (0x02, 0x180),
        (0x2a, 0x004),
        (0x03, 0x00f),
        (0x01, 0x02f),
        (0x04, 0x010),
        (0x06, 0x001),
    ];
    for (i, v) in pll.iter().enumerate() {
        w.push((0x24 + i as u8, *v));
    }
    w.push((0x06, 0x100 | (div << 5) | 0x004 | 0x001));
    w.push((0x07, (sr << 1) | 1));
    w.extend([(0x32, 0x001), (0x33, 0x001), (0x2a, 0x000)]);
    w.extend([(0x01, 0x02e), (0x3d, 0x000), (0x0a, 0x008)]);
    w.extend([(0x0b, 0x0ff), (0x0c, 0x1ff), (0x34, 0x0b9), (0x35, 0x1b9)]);
    w
}

/// Frames per segment: a whole number of 1 kHz periods, so the ring wraps without a phase step,
/// and at least 10 ms.
fn segment_frames(rate: u32) -> u32 {
    let unit = rate / gcd(rate, 1000);
    unit * (rate / 100 / unit).max(1)
}

fn plan(rate: u32) -> Plan {
    let seg_frames = segment_frames(rate);
    let total_segs = (0.3 * f64::from(rate) / f64::from(seg_frames)).ceil() as u32;
    Plan {
        rate,
        regs: bring_up(rate),
        seg_frames,
        total_segs: total_segs.max(RING_SEGS * 2 + 1),
    }
}

/// One cycle of the ring, as the guest will fetch it: left is a sine, right a cosine at half
/// amplitude, so a swapped or shifted channel cannot match by accident.
fn ring(p: &Plan) -> Vec<(i16, i16)> {
    (0..p.seg_frames * RING_SEGS)
        .map(|n| {
            let ph = 2.0 * std::f64::consts::PI * TONE_HZ * f64::from(n) / f64::from(p.rate);
            (
                (16000.0 * ph.sin()).round() as i16,
                (8000.0 * ph.cos()).round() as i16,
            )
        })
        .collect()
}

fn put32(img: &mut [u8], at: u32, v: u32) {
    img[at as usize..at as usize + 4].copy_from_slice(&v.to_le_bytes());
}

fn image(p: &Plan) -> Vec<u8> {
    let seg_bytes = p.seg_frames * 4;
    let ring_bytes = seg_bytes * RING_SEGS;
    let mut img = vec![0u8; (RING + ring_bytes) as usize];
    let code = parse_words(CODE).unwrap();
    img[..code.len()].copy_from_slice(&code);
    assert!(code.len() <= PARAMS as usize);

    // DMA0_CMD: request id 2, wait for request, single shot, RAM to peripheral, interrupt, start.
    let cmd =
        (2 << 16) | (1 << 24) | (1 << 26) | (1 << 27) | (1 << 30) | (1 << 31) | (seg_bytes - 4);
    for (i, v) in [
        PHYS + RING,
        PHYS + RING + ring_bytes,
        seg_bytes,
        cmd,
        p.total_segs,
        PHYS + RING,
        0,
        0,
        TABLE,
    ]
    .iter()
    .enumerate()
    {
        put32(&mut img, PARAMS + 4 * i as u32, *v);
    }
    assert!(p.regs.len() < 64);
    for (i, (reg, val)) in p.regs.iter().enumerate() {
        let b0 = (u32::from(*reg) << 1) | u32::from(val >> 8);
        put32(
            &mut img,
            TABLE + 4 * i as u32,
            b0 | (u32::from(*val & 0xff) << 8),
        );
    }
    put32(&mut img, TABLE + 4 * p.regs.len() as u32, 0xffff_ffff);
    for (n, (l, r)) in ring(p).iter().enumerate() {
        let w = u32::from(*l as u16) | (u32::from(*r as u16) << 16);
        put32(&mut img, RING + 4 * n as u32, w);
    }
    img
}

fn machine(img: &[u8]) -> Machine {
    let mut m = Machine::new(&EApp::none(), 0x1100_0000, 0x0100_0000);
    map_hardware(&mut m, false);
    m.mem.i2c_base = Some(0x7000_c000);
    for (i, b) in img.iter().enumerate() {
        m.mem.write8(i as u32, *b);
    }
    m.cpu.regs[15] = 0;
    m
}

fn word(m: &mut Machine, off: u32) -> u32 {
    m.mem.read32(PARAMS + off)
}

fn out_path(name: &str) -> PathBuf {
    let dir = PathBuf::from(env!("CARGO_TARGET_TMPDIR"));
    std::fs::create_dir_all(&dir).unwrap();
    dir.join(name)
}

/// Run until the guest sets its done flag, in slices so a hang fails rather than blocks.
fn run_to_done(m: &mut Machine) -> bool {
    for _ in 0..120 {
        m.run(500_000);
        if word(m, 24) != 0 {
            return true;
        }
    }
    false
}

/// Rising zero crossings of `left`, located by linear interpolation, as a frequency.
fn tone_hz(left: &[i16], rate: u32) -> f64 {
    let mut times = Vec::new();
    for (i, w) in left.windows(2).enumerate() {
        let (a, b) = (f64::from(w[0]), f64::from(w[1]));
        if a < 0.0 && b >= 0.0 {
            times.push(i as f64 + (-a) / (b - a));
        }
    }
    assert!(times.len() > 10, "{} crossings", times.len());
    let cycles = (times.len() - 1) as f64;
    cycles * f64::from(rate) / (times[times.len() - 1] - times[0])
}

fn play(rate: u32) {
    let p = plan(rate);
    let img = image(&p);
    let mut m = machine(&img);
    let wav = out_path(&format!("tone-{rate}.wav"));
    let _ = std::fs::remove_file(&wav);
    m.mem.enable_audio(&wav, Wm8758::DEFAULT_MCLK_HZ);
    assert!(
        run_to_done(&mut m),
        "{rate}: the guest never finished; pc {:#x}",
        m.cpu.regs[15]
    );

    let files = m.mem.audio.as_mut().unwrap().finish().unwrap();
    assert_eq!(
        files,
        vec![wav.clone()],
        "{rate}: exactly one rate was played"
    );
    let (got_rate, pcm) = parse_wav(&std::fs::read(&wav).unwrap()).expect("a canonical WAV");
    assert_eq!(
        got_rate, rate,
        "rate decoded from the guest's PLL and divider writes"
    );

    let frames = (p.total_segs * p.seg_frames) as usize;
    assert_eq!(pcm.len(), frames * 2, "{rate}: length");
    let ring = ring(&p);
    for (n, f) in pcm.chunks(2).enumerate() {
        assert_eq!((f[0], f[1]), ring[n % ring.len()], "{rate}: frame {n}");
    }
    let left: Vec<i16> = pcm.chunks(2).map(|f| f[0]).collect();
    let hz = tone_hz(&left, got_rate);
    assert!((hz - TONE_HZ).abs() < 0.5, "{rate}: decoded {hz} Hz");

    // One FIQ per segment, delivered as FIQ and not IRQ, counted by the guest and the machine.
    assert_eq!(word(&mut m, 28), p.total_segs, "{rate}: guest FIQ count");
    assert_eq!(m.fiqs_taken, u64::from(p.total_segs), "{rate}: FIQs taken");
    assert_eq!(m.irqs_taken, 0, "{rate}: nothing arrived as IRQ");

    // The DMA is paced at the decoded rate, so the run took as long as the audio does.
    let secs = frames as f64 / f64::from(rate);
    let ran = f64::from(m.mem.usec) / 1e6;
    assert!(
        ran >= secs && ran < secs * 1.05 + 0.01,
        "{rate}: {ran} s simulated for {secs} s of audio"
    );
    let a = m.mem.audio.as_ref().unwrap();
    assert_eq!(
        (a.frames, a.dropped_fifo_disabled, a.dropped_unclocked),
        (frames as u64, 0, 0)
    );
    assert_eq!(
        a.frames_dac_off, 0,
        "R3 enabled both DACs before the first frame"
    );
}

#[test]
fn a_1khz_tone_comes_out_at_44100() {
    play(44100);
}

#[test]
fn a_1khz_tone_comes_out_at_48000() {
    play(48000);
}

#[test]
fn every_rate_the_sound_layer_offers_plays_at_its_own_rate() {
    for rate in Wm8758::RATES {
        play(rate);
    }
}

#[test]
fn a_guest_that_never_clocks_the_codec_is_paced_but_records_nothing() {
    let mut p = plan(44100);
    p.regs = vec![(0x03, 0x00f), (0x04, 0x010)];
    let mut m = machine(&image(&p));
    let wav = out_path("unclocked.wav");
    let _ = std::fs::remove_file(&wav);
    m.mem.enable_audio(&wav, Wm8758::DEFAULT_MCLK_HZ);
    assert!(run_to_done(&mut m));
    let a = m.mem.audio.as_mut().unwrap();
    assert_eq!(a.frames, 0);
    assert_eq!(a.dropped_unclocked, u64::from(p.total_segs * p.seg_frames));
    assert!(a.finish().unwrap().is_empty());
    assert!(!wav.exists(), "no frame, no file");
}

#[test]
fn with_the_sink_off_nothing_about_the_machine_changes() {
    let p = plan(44100);
    let mut m = machine(&image(&p));
    // Not enabled: the DMA still completes at the old fixed pace, but priority routing is off, so
    // the completion arrives as IRQ, lands on the guest's `b .` IRQ vector and the guest stops.
    m.run(3_000_000);
    assert!(m.mem.audio.is_none() && !m.mem.fiq_routing);
    assert_eq!(
        (m.fiqs_taken, word(&mut m, 28), word(&mut m, 24)),
        (0, 0, 0)
    );
    assert!(
        m.mem.pp_dma_transfers >= 1,
        "the first segment was still moved"
    );
    assert_eq!(m.mem.i2s_paced_transfers, 1);
}

#[test]
fn cpu_stores_to_the_fifo_port_reach_the_sink_and_the_free_count_reads_empty() {
    let p = plan(48000);
    let mut m = machine(&image(&Plan { total_segs: 1, ..p }));
    let wav = out_path("pio.wav");
    m.mem.enable_audio(&wav, Wm8758::DEFAULT_MCLK_HZ);
    assert!(run_to_done(&mut m));
    let before = m.mem.audio.as_ref().unwrap().frames;
    m.mem.write32(0x7000_2840, 0x0002_0001);
    m.mem.write8(0x7000_2840, 3);
    m.mem.write8(0x7000_2841, 0);
    m.mem.write8(0x7000_2842, 4);
    m.mem.write8(0x7000_2843, 0);
    assert_eq!(m.mem.audio.as_ref().unwrap().frames, before + 2);
    let cfg = m.mem.read32(0x7000_280c);
    assert_eq!((cfg >> 16) & 0x3f, 16, "TX free count: empty");
    assert_eq!(cfg & 0x3f, 1, "the guest's request level survives the OR");
    let files = m.mem.audio.as_mut().unwrap().finish().unwrap();
    let (rate, pcm) = parse_wav(&std::fs::read(&files[0]).unwrap()).unwrap();
    assert_eq!(rate, 48000);
    assert_eq!(&pcm[pcm.len() - 4..], &[1, 2, 3, 4]);
}

/// The guest image is also written out, so the same bytes can be booted through `trace` and
/// `tools/emu/mkdrive.pl`: `trace ... --osos-from-disk --boot-osos --disk=DRIVE --audio-wav=FILE`.
#[test]
fn the_guest_image_is_kept_for_the_trace_boot() {
    for rate in [44100, 48000] {
        let img = image(&plan(rate));
        let path = out_path(&format!("audio-tone-{rate}.bin"));
        std::fs::write(&path, &img).unwrap();
        // mkdrive finds an image by two leading branch words.
        assert_eq!(img[3], 0xea);
        assert_eq!(img[7], 0xea);
    }
}

/// Run the guest the way the window does: in slices, taking the queued live frames after each.
fn run_live(m: &mut Machine) -> Vec<ipod_machine::LiveChunk> {
    let mut out = Vec::new();
    for _ in 0..120 {
        m.run(250_000);
        out.extend(m.mem.take_audio());
        if word(m, 24) != 0 {
            return out;
        }
    }
    panic!("the guest never finished; pc {:#x}", m.cpu.regs[15]);
}

fn flatten(chunks: &[ipod_machine::LiveChunk]) -> Vec<f32> {
    chunks.iter().flat_map(|c| c.pcm.iter().copied()).collect()
}

fn ring_f32(p: &Plan, gain: (f32, f32)) -> Vec<f32> {
    let one = ring(p);
    let total = (p.total_segs * p.seg_frames) as usize;
    (0..total)
        .flat_map(|n| {
            let (l, r) = one[n % one.len()];
            [
                f32::from(l) / 32768.0 * gain.0,
                f32::from(r) / 32768.0 * gain.1,
            ]
        })
        .collect()
}

#[test]
fn live_frames_are_the_frames_the_wav_gets_and_the_wav_is_unchanged() {
    for rate in [44100, 48000] {
        let p = plan(rate);
        let img = image(&p);

        let wav_only = out_path(&format!("live-off-{rate}.wav"));
        let _ = std::fs::remove_file(&wav_only);
        let mut a = machine(&img);
        a.mem.enable_audio(&wav_only, Wm8758::DEFAULT_MCLK_HZ);
        assert!(run_live(&mut a).is_empty(), "no queue unless asked for");
        a.mem.audio.as_mut().unwrap().finish().unwrap();

        let wav_both = out_path(&format!("live-on-{rate}.wav"));
        let _ = std::fs::remove_file(&wav_both);
        let mut b = machine(&img);
        b.mem.enable_audio(&wav_both, Wm8758::DEFAULT_MCLK_HZ);
        b.mem.enable_audio_live(Wm8758::DEFAULT_MCLK_HZ);
        let chunks = run_live(&mut b);
        b.mem.audio.as_mut().unwrap().finish().unwrap();

        assert_eq!(
            std::fs::read(&wav_only).unwrap(),
            std::fs::read(&wav_both).unwrap(),
            "{rate}: the WAV bytes do not depend on the live queue"
        );
        assert_eq!(
            (a.mem.usec, a.fiqs_taken, a.executed),
            (b.mem.usec, b.fiqs_taken, b.executed),
            "{rate}: neither does the machine's time"
        );
        assert!(chunks.iter().all(|c| c.rate == rate), "{rate}");
        // Volume 0 dB on the DAC and OUT1 in the guest's bring-up, so the gain is exactly 1.
        assert_eq!(flatten(&chunks), ring_f32(&p, (1.0, 1.0)), "{rate}");
        assert_eq!(b.mem.audio.as_ref().unwrap().live_dropped(), 0);
    }
}

#[test]
fn live_only_writes_no_file_and_keeps_no_pcm() {
    let p = plan(44100);
    let mut m = machine(&image(&p));
    m.mem.enable_audio_live(Wm8758::DEFAULT_MCLK_HZ);
    let chunks = run_live(&mut m);
    assert_eq!(flatten(&chunks), ring_f32(&p, (1.0, 1.0)));
    let a = m.mem.audio.as_mut().unwrap();
    assert!(a.finish().unwrap().is_empty());
    assert!(a.take_live().is_empty(), "taken once, gone");
    assert_eq!(m.fiqs_taken, u64::from(p.total_segs));
}

#[test]
fn live_frames_follow_soft_mute_and_the_volume_registers() {
    // Soft mute written last: every frame is silent, the WAV path would still hold the tone.
    let mut p = plan(44100);
    p.regs.push((0x0a, 0x040));
    let mut m = machine(&image(&p));
    m.mem.enable_audio_live(Wm8758::DEFAULT_MCLK_HZ);
    let got = flatten(&run_live(&mut m));
    assert_eq!(got.len(), (p.total_segs * p.seg_frames * 2) as usize);
    assert!(got.iter().all(|&s| s == 0.0));

    // The sound layer's 10% setting: DAC 207, PGA 0, which is -81 dB on both channels.
    let mut p = plan(44100);
    p.regs
        .extend([(0x0b, 0x0cf), (0x0c, 0x1cf), (0x34, 0x080), (0x35, 0x180)]);
    let mut m = machine(&image(&p));
    m.mem.enable_audio_live(Wm8758::DEFAULT_MCLK_HZ);
    let got = flatten(&run_live(&mut m));
    let g = 10f32.powf(-81.0 / 20.0);
    let want = ring_f32(&p, (g, g));
    assert_eq!(got.len(), want.len());
    for (n, (a, b)) in got.iter().zip(&want).enumerate() {
        assert!((a - b).abs() < 1e-9, "sample {n}: {a} against {b}");
    }

    // Left and right are separate: right DAC volume 0 mutes the right channel only.
    let mut p = plan(44100);
    p.regs.push((0x0c, 0x100));
    let mut m = machine(&image(&p));
    m.mem.enable_audio_live(Wm8758::DEFAULT_MCLK_HZ);
    let got = flatten(&run_live(&mut m));
    let want = ring_f32(&p, (1.0, 0.0));
    assert_eq!(got, want);
}
