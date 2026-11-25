//! The WM8758 and I2S sink through the bus, with no guest code: codec registers written the way
//! the I2C controller delivers them, FIFO words stored the way a CPU or the DMA engine stores them.

use arm7tdmi::Bus as _;
use ipod_machine::hw::audio::parse_wav;
use ipod_machine::{map_hardware, EApp, Machine, Wm8758};
use std::path::PathBuf;

const IISCONFIG: u32 = 0x7000_2800;
const IISFIFO_CFG: u32 = 0x7000_280c;
const IISFIFO_WR: u32 = 0x7000_2840;

fn out_path(name: &str) -> PathBuf {
    let dir = PathBuf::from(env!("CARGO_TARGET_TMPDIR"));
    std::fs::create_dir_all(&dir).unwrap();
    dir.join(name)
}

fn machine() -> Machine {
    let mut m = Machine::new(&EApp::none(), 0x1100_0000, 0x0100_0000);
    map_hardware(&mut m, false);
    m
}

/// The clock select and the PLL, divider and power writes that make the codec the 48 kHz I2S
/// master, as Rockbox makes them.
fn clock_codec_at_48k(m: &mut Machine) {
    m.mem.write32(0x7000_0018, 0);
    for (reg, val) in [
        (0x01u8, 0x02fu16),
        (0x03, 0x00f),
        (0x06, 0x001),
        (0x24, 0x018),
        (0x25, 0x00c),
        (0x26, 0x093),
        (0x27, 0x0e9),
        (0x06, 0x145),
    ] {
        m.mem
            .wm8758
            .transfer(0x82, [(reg << 1) | (val >> 8) as u8, val as u8, 0, 0]);
    }
}

#[test]
fn stores_to_the_fifo_port_reach_the_sink_and_the_free_count_reads_empty() {
    let mut m = machine();
    let wav = out_path("pio.wav");
    let _ = std::fs::remove_file(&wav);
    m.mem.enable_audio(&wav, Wm8758::DEFAULT_MCLK_HZ);
    clock_codec_at_48k(&mut m);
    m.mem.write32(IISCONFIG, 1 << 29);
    m.mem.write32(IISFIFO_CFG, 1);

    m.mem.write32(IISFIFO_WR, 0x0002_0001);
    m.mem.write8(IISFIFO_WR, 3);
    m.mem.write8(IISFIFO_WR + 1, 0);
    m.mem.write8(IISFIFO_WR + 2, 4);
    m.mem.write8(IISFIFO_WR + 3, 0);
    assert_eq!(m.mem.audio.as_ref().unwrap().frames, 2);
    assert_eq!(m.mem.audio_rate(), Some(48000));

    let cfg = m.mem.read32(IISFIFO_CFG);
    assert_eq!((cfg >> 16) & 0x3f, 16, "TX free count: empty");
    assert_eq!(
        cfg & 0x3f,
        1,
        "the request level the guest wrote survives the OR"
    );

    let files = m.mem.audio.as_mut().unwrap().finish().unwrap();
    assert_eq!(files, vec![wav.clone()]);
    let (rate, pcm) = parse_wav(&std::fs::read(&wav).unwrap()).unwrap();
    assert_eq!((rate, pcm.as_slice()), (48000, &[1, 2, 3, 4][..]));
}

#[test]
fn frames_are_dropped_until_the_fifo_is_enabled_and_the_codec_is_clocking() {
    let mut m = machine();
    let wav = out_path("gated.wav");
    let _ = std::fs::remove_file(&wav);
    m.mem.enable_audio(&wav, Wm8758::DEFAULT_MCLK_HZ);

    // Neither: TXFIFOEN clear, codec at reset.
    m.mem.write32(IISFIFO_WR, 1);
    // TXFIFOEN set, codec still the I2S slave.
    m.mem.write32(IISCONFIG, 1 << 29);
    m.mem.write32(IISFIFO_WR, 2);
    // Codec clocking, TXFIFOEN clear again.
    clock_codec_at_48k(&mut m);
    m.mem.write32(IISCONFIG, 0);
    m.mem.write32(IISFIFO_WR, 3);

    let a = m.mem.audio.as_mut().unwrap();
    assert_eq!(
        (
            a.words,
            a.frames,
            a.dropped_fifo_disabled,
            a.dropped_unclocked
        ),
        (3, 0, 2, 1)
    );
    assert!(a.finish().unwrap().is_empty());
    assert!(!wav.exists(), "no frame, no file");
}

#[test]
fn with_the_sink_off_the_fifo_port_is_plain_storage() {
    let mut m = machine();
    m.mem.write32(IISFIFO_WR, 0x1234_5678);
    assert!(m.mem.audio.is_none());
    assert_eq!(m.mem.read32(IISFIFO_WR), 0x1234_5678);
    assert_eq!((m.mem.read32(IISFIFO_CFG) >> 16) & 0x3f, 0);
}

/// RetailOS's playback-time codec driver: both bytes of a write in one word at the first data
/// register, with a stale byte left in the second by an earlier transfer.
#[test]
fn a_word_stored_to_the_first_i2c_data_register_sends_its_second_byte() {
    const I2C: u32 = 0x7000_c000;
    let mut m = machine();
    m.mem.i2c_base = Some(I2C);
    m.mem.write8(I2C + 0x10, 0x01);
    m.mem.write32(I2C + 4, u32::from(Wm8758::ADDR) << 1);
    m.mem.write32(I2C, 0x02);
    m.mem.write32(I2C + 0x0c, 0x0000_4d0d);
    m.mem.write32(I2C, 0x82);
    assert_eq!(m.mem.wm8758.reg(0x06), Some(0x14d));

    // One byte per register still wins when the second register is stored after the first.
    m.mem.write8(I2C + 0x0c, 0x68);
    m.mem.write8(I2C + 0x10, 0x40);
    m.mem.write8(I2C, 0x82);
    assert_eq!(m.mem.wm8758.reg(0x34), Some(0x040));
}

/// RetailOS's 44.1 kHz PLL constants, on the clock select it starts with and on Rockbox's.
#[test]
fn the_external_clock_select_halves_the_codec_clock() {
    let mut m = machine();
    m.mem.write32(0x7000_0018, 0x0000_0008);
    m.mem.enable_audio(out_path("ext_clock.wav"), Wm8758::DEFAULT_MCLK_HZ);
    for (reg, val) in [
        (0x01u8, 0x02du16),
        (0x24, 0x007),
        (0x25, 0x021),
        (0x26, 0x151),
        (0x27, 0x1ac),
        (0x06, 0x14d),
    ] {
        m.mem
            .wm8758
            .transfer(0x82, [(reg << 1) | (val >> 8) as u8, val as u8, 0, 0]);
    }
    assert_eq!(m.mem.audio_rate(), Some(44_100));
    m.mem.write32(0x7000_0018, 0);
    assert_eq!(m.mem.audio_rate(), None, "88.2 kHz from 24 MHz is outside the part's range");
}
