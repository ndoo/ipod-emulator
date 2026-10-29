//! Host audio output for the emulator window.
//!
//! The machine thread produces stereo frames at whatever rate the codec model decodes (8 to 48 kHz)
//! in bursts, once per interpreter slice, and the device wants a steady stream at its own rate. The
//! two meet in a ring buffer:
//!
//! ```text
//! machine thread                                         device thread
//! Feeder::push(rate, frames)  -->  resample  -->  ring  -->  Player::fill(out, channels)
//! Feeder::throttle()
//! ```
//!
//! [`Feeder`] and [`Player`] hold no device. The window opens one with [`open`] and hands the
//! `Player` to the device callback; a test hands it to a loop that stands in for the device.
//!
//! # When the machine is not in step with the device
//!
//! * **Slower than real time (underrun).** The ring runs dry and the callback writes silence, not a
//!   repeat of the last samples. It then stays silent until [`Config::preroll_ms`] of audio has
//!   built up again, so a machine that is only a little short does not turn into a train of
//!   fragments. No sample is skipped or played twice across the gap; what arrives is played in
//!   order. [`Stats::underruns`] counts the gaps and [`Stats::silence_frames`] the silence.
//! * **Faster than real time (overrun).** The machine thread is not paced by anything else, so
//!   [`Feeder::throttle`] makes it wait while more than [`Config::high_ms`] is queued. That holds
//!   the machine at real time while a track plays, which is what the part does. If the device
//!   stops draining (a stalled or suspended stream) the wait ends after [`Config::stall_ms`] and
//!   the queue is allowed to fill; frames that no longer fit are dropped, newest first, and counted
//!   in [`Stats::dropped_frames`]. The machine is never blocked for longer than that.

mod device;
mod resample;

pub use device::{open, Device};
pub use resample::Resampler;

use rtrb::{Consumer, Producer, RingBuffer};
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::Duration;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Config {
    /// Capacity of the ring, in milliseconds of device audio.
    pub ring_ms: u32,
    /// Audio that has to be queued before the callback starts, or restarts after an underrun.
    pub preroll_ms: u32,
    /// [`Feeder::throttle`] waits while more than this is queued...
    pub high_ms: u32,
    /// ...until no more than this is.
    pub low_ms: u32,
    /// [`Feeder::throttle`] gives up when the queue has not shrunk for this long.
    pub stall_ms: u32,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            ring_ms: 500,
            preroll_ms: 50,
            high_ms: 250,
            low_ms: 150,
            stall_ms: 250,
        }
    }
}

/// Counters shared by both ends. Read them with [`Stats::snapshot`].
#[derive(Debug, Default)]
pub struct Stats {
    underruns: AtomicU64,
    silence_frames: AtomicU64,
    dropped_frames: AtomicU64,
    played_frames: AtomicU64,
    pushed_frames: AtomicU64,
}

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct Snapshot {
    /// Times the callback found the ring short after it had started playing. A pause counts once.
    pub underruns: u64,
    /// Device frames written as silence since the first frame was played, pauses included.
    pub silence_frames: u64,
    /// Device frames that arrived while the ring was full and were discarded.
    pub dropped_frames: u64,
    /// Device frames the callback took from the ring.
    pub played_frames: u64,
    /// Device frames the feeder put in the ring.
    pub pushed_frames: u64,
}

impl Stats {
    pub fn snapshot(&self) -> Snapshot {
        Snapshot {
            underruns: self.underruns.load(Ordering::Relaxed),
            silence_frames: self.silence_frames.load(Ordering::Relaxed),
            dropped_frames: self.dropped_frames.load(Ordering::Relaxed),
            played_frames: self.played_frames.load(Ordering::Relaxed),
            pushed_frames: self.pushed_frames.load(Ordering::Relaxed),
        }
    }
}

impl Snapshot {
    pub fn report(&self, device_rate: u32) -> String {
        let secs = |f: u64| f as f64 / f64::from(device_rate.max(1));
        format!(
            "audio: {:.2} s played, {} underruns, {:.2} s of silence since the first sample, {:.2} s dropped",
            secs(self.played_frames),
            self.underruns,
            secs(self.silence_frames),
            secs(self.dropped_frames),
        )
    }
}

/// The machine-thread end.
pub struct Feeder {
    prod: Producer<f32>,
    resampler: Resampler,
    scratch: Vec<f32>,
    device_rate: u32,
    capacity_frames: usize,
    cfg: Config,
    stats: Arc<Stats>,
    stalled: bool,
}

/// The device-thread end. [`Player::fill`] does not allocate, lock or block.
pub struct Player {
    cons: Consumer<f32>,
    primed: bool,
    started: bool,
    preroll_frames: usize,
    stats: Arc<Stats>,
}

/// A connected [`Feeder`] and [`Player`] for a device that runs at `device_rate`.
pub fn pair(device_rate: u32, cfg: Config) -> (Feeder, Player, Arc<Stats>) {
    let ms = |ms: u32| (u64::from(device_rate) * u64::from(ms) / 1000) as usize;
    let capacity_frames = ms(cfg.ring_ms).max(2);
    let (prod, cons) = RingBuffer::new(capacity_frames * 2);
    let stats = Arc::new(Stats::default());
    (
        Feeder {
            prod,
            resampler: Resampler::new(device_rate, device_rate),
            scratch: Vec::new(),
            device_rate,
            capacity_frames,
            cfg,
            stats: stats.clone(),
            stalled: false,
        },
        Player {
            cons,
            primed: false,
            started: false,
            preroll_frames: ms(cfg.preroll_ms).min(capacity_frames),
            stats: stats.clone(),
        },
        stats,
    )
}

impl Feeder {
    pub fn device_rate(&self) -> u32 {
        self.device_rate
    }

    /// Frames waiting in the ring, in device frames.
    pub fn queued_frames(&self) -> usize {
        self.capacity_frames - self.prod.slots() / 2
    }

    /// Queue interleaved stereo `pcm` that runs at `rate`. Frames that do not fit are dropped.
    pub fn push(&mut self, rate: u32, pcm: &[f32]) {
        if rate == 0 || pcm.len() < 2 {
            return;
        }
        if self.resampler.in_rate() != rate {
            self.resampler = Resampler::new(rate, self.device_rate);
        }
        self.scratch.clear();
        self.resampler.process(pcm, &mut self.scratch);
        let frames = self.scratch.len() / 2;
        let room = self.prod.slots() / 2;
        let take = frames.min(room);
        if take > 0 {
            if let Ok(mut chunk) = self.prod.write_chunk(take * 2) {
                let (a, b) = chunk.as_mut_slices();
                let head = a.len();
                a.copy_from_slice(&self.scratch[..head]);
                b.copy_from_slice(&self.scratch[head..take * 2]);
                chunk.commit_all();
            }
        }
        self.stats
            .pushed_frames
            .fetch_add(take as u64, Ordering::Relaxed);
        if frames > take {
            self.stats
                .dropped_frames
                .fetch_add((frames - take) as u64, Ordering::Relaxed);
        }
    }

    /// Move what the machine's I2S sink has queued into the ring, then [`Feeder::throttle`]. The
    /// window calls this after every interpreter slice.
    pub fn pump(&mut self, mem: &mut ipod_machine::Memory, quit: impl Fn() -> bool) {
        for chunk in mem.take_audio() {
            self.push(chunk.rate, &chunk.pcm);
        }
        self.throttle(quit);
    }

    /// Hold the calling thread while the device is more than [`Config::high_ms`] ahead, sleeping
    /// in 2 ms steps until it is within [`Config::low_ms`], `quit` returns true, or the queue has
    /// stopped shrinking for [`Config::stall_ms`]. See the crate documentation.
    pub fn throttle(&mut self, quit: impl Fn() -> bool) {
        self.throttle_with(quit, std::thread::sleep);
    }

    /// [`Feeder::throttle`] with the sleep supplied, so a test can advance a fake device by the
    /// time slept.
    pub fn throttle_with(&mut self, quit: impl Fn() -> bool, mut sleep: impl FnMut(Duration)) {
        const STEP: Duration = Duration::from_millis(2);
        let frames = |ms: u32| (u64::from(self.device_rate) * u64::from(ms) / 1000) as usize;
        let (high, low) = (frames(self.cfg.high_ms), frames(self.cfg.low_ms));
        if self.stalled {
            if self.queued_frames() <= low {
                self.stalled = false;
            }
            return;
        }
        if self.queued_frames() <= high {
            return;
        }
        let mut best = self.queued_frames();
        let mut stuck = Duration::ZERO;
        let limit = Duration::from_millis(u64::from(self.cfg.stall_ms));
        while self.queued_frames() > low && !quit() {
            sleep(STEP);
            let now = self.queued_frames();
            if now < best {
                best = now;
                stuck = Duration::ZERO;
            } else {
                stuck += STEP;
                if stuck >= limit {
                    self.stalled = true;
                    return;
                }
            }
        }
    }
}

impl Player {
    /// Fill `out`, interleaved with `channels` channels, from the ring. The first two channels get
    /// left and right; one channel gets their mean; any others are silent.
    pub fn fill(&mut self, out: &mut [f32], channels: usize) {
        let channels = channels.max(1);
        let want = out.len() / channels;
        let have = self.cons.slots() / 2;
        if !self.primed {
            if have < self.preroll_frames.max(1) {
                self.silence(out, want);
                return;
            }
            self.primed = true;
            self.started = true;
        }
        let take = want.min(have);
        if take > 0 {
            if let Ok(chunk) = self.cons.read_chunk(take * 2) {
                let (a, b) = chunk.as_slices();
                let mut frame = 0;
                for pair in a.chunks_exact(2).chain(b.chunks_exact(2)) {
                    let o = &mut out[frame * channels..(frame + 1) * channels];
                    match channels {
                        1 => o[0] = (pair[0] + pair[1]) * 0.5,
                        _ => {
                            o[0] = pair[0];
                            o[1] = pair[1];
                            o[2..].fill(0.0);
                        }
                    }
                    frame += 1;
                }
                chunk.commit_all();
            }
            self.stats
                .played_frames
                .fetch_add(take as u64, Ordering::Relaxed);
        }
        if take < want {
            self.primed = false;
            self.stats.underruns.fetch_add(1, Ordering::Relaxed);
            self.silence(&mut out[take * channels..], want - take);
        }
    }

    fn silence(&mut self, out: &mut [f32], frames: usize) {
        out.fill(0.0);
        if self.started {
            self.stats
                .silence_frames
                .fetch_add(frames as u64, Ordering::Relaxed);
        }
    }
}

#[cfg(test)]
mod tests;
