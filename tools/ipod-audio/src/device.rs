use crate::{Config, Feeder, Stats};
use std::sync::Arc;

/// An open output stream. Dropping it closes the stream. It may not be `Send`, so it has to be
/// dropped on the thread that opened it.
pub struct Device {
    pub name: String,
    pub rate: u32,
    pub channels: u16,
    pub stats: Arc<Stats>,
    _stream: Stream,
}

#[cfg(any(target_os = "macos", target_os = "windows"))]
type Stream = cpal::Stream;
#[cfg(not(any(target_os = "macos", target_os = "windows")))]
type Stream = ();

/// Open the default output device at its default configuration and return it with the feeder for
/// its ring.
#[cfg(any(target_os = "macos", target_os = "windows"))]
pub fn open(cfg: Config) -> Result<(Device, Feeder), String> {
    use crate::pair;
    use cpal::traits::{DeviceTrait, HostTrait, StreamTrait};
    use cpal::{FromSample, SampleFormat, SizedSample};

    let host = cpal::default_host();
    let device = host
        .default_output_device()
        .ok_or("no default audio output device")?;
    let name = device
        .description()
        .map(|d| d.name().to_string())
        .unwrap_or_else(|_| "default output".into());
    let supported = device
        .default_output_config()
        .map_err(|e| format!("{name}: no default output configuration: {e}"))?;
    let format = supported.sample_format();
    let config: cpal::StreamConfig = supported.into();
    let (rate, channels) = (config.sample_rate, config.channels);
    let (feeder, mut player, stats) = pair(rate, cfg);

    fn callback<T: SizedSample + FromSample<f32>>(
        mut player: crate::Player,
        channels: usize,
    ) -> impl FnMut(&mut [T], &cpal::OutputCallbackInfo) + Send + 'static {
        // Sized for the largest buffer seen so far; it grows on the first oversize callback only.
        let mut scratch: Vec<f32> = Vec::with_capacity(16384);
        move |data: &mut [T], _| {
            scratch.resize(data.len(), 0.0);
            player.fill(&mut scratch, channels);
            for (d, s) in data.iter_mut().zip(&scratch) {
                *d = T::from_sample_(*s);
            }
        }
    }
    let on_error = |e| eprintln!("audio: stream error: {e}");
    let ch = usize::from(channels);
    let stream = match format {
        SampleFormat::F32 => device.build_output_stream(
            config,
            move |data: &mut [f32], _: &cpal::OutputCallbackInfo| player.fill(data, ch),
            on_error,
            None,
        ),
        SampleFormat::I16 => {
            device.build_output_stream(config, callback::<i16>(player, ch), on_error, None)
        }
        SampleFormat::U16 => {
            device.build_output_stream(config, callback::<u16>(player, ch), on_error, None)
        }
        SampleFormat::I32 => {
            device.build_output_stream(config, callback::<i32>(player, ch), on_error, None)
        }
        other => return Err(format!("{name}: sample format {other} is not handled")),
    }
    .map_err(|e| format!("{name}: cannot open the output stream: {e}"))?;
    stream
        .play()
        .map_err(|e| format!("{name}: cannot start the output stream: {e}"))?;
    Ok((
        Device {
            name,
            rate,
            channels,
            stats: stats.clone(),
            _stream: stream,
        },
        feeder,
    ))
}

#[cfg(not(any(target_os = "macos", target_os = "windows")))]
pub fn open(_cfg: Config) -> Result<(Device, Feeder), String> {
    Err("this build has no host audio backend (macOS and Windows only)".into())
}
