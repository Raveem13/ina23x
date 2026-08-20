//========== INA239 tests using embedded_hal_mock ==========

//---------- Imports ----------
// Import driver structures, functions from the parent module
use ina23x::{AdcRange::*, Count::*, Mode::*, Time::*, *};

// std library for testing
extern crate std;

// Embedded-hal-mock library for testing SPI communication
use embedded_hal_mock::eh1::spi::{Mock as SpiMock, Transaction as SpiTransaction};

//---------- Constants ----------

//---------- Tests ----------

//Test for default instantiation
#[test]
fn default() {
    let mut spi = SpiMock::new(&[]);
    INA239::new(spi.clone());
    spi.done();
}

// Reset test
#[test]
fn reset_test() {
    let expectations = [
        write_txn(0x00, 1 << 15), 
        read_txn(0x05, &[0x00, 0x00]),
    ];
    let mut spi = SpiMock::new(&expectations);
    let mut ina = INA239::new(spi.clone());
    // ina.reset().unwrap();
    //Check busvoltage after reset
    assert_eq!(ina.bus_voltage().unwrap(), 0.0);
    spi.done();
}

// Test to get manufacture ID
#[test]
fn test_manufacture_id() {
    let mut spi = SpiMock::new(&[read_txn(0x3E, &[0x54, 0x49])]);
    let mut ina = INA239::new(spi.clone());
    let id = ina.manufacture_id().unwrap();
    assert_eq!(id, 0x5449);
    spi.done();
}

// Test to get device ID
#[test]
fn test_device_id() {
    let mut spi = SpiMock::new(&[read_txn(0x3F, &[0x23, 0x91])]);
    let mut ina = INA239::new(spi.clone());
    let did = ina.device_id().unwrap();
    assert_eq!(did, (0x238, 0x1));
    spi.done();
}

// Calibration test
#[test]
fn test_calibration() {
    let shunt_cal = exp_shunt_cal(20.0, 0.1, false);
    let mut spi = SpiMock::new(&[write_txn(0x02, shunt_cal)]);
    let mut ina = INA239::new(spi.clone());
    ina.set_shunt_calibrate(20.0, 0.1).unwrap();
    spi.done();
}

// Configuration test
#[test]
fn test_configuration() {
    let confg_val = exp_confg(10, true);
    let mut spi = SpiMock::new(&[write_txn(0x00, confg_val)]);
    let mut ina = INA239::new(spi.clone());
    ina.set_config(10, Range40mV).unwrap();
    spi.done();
}

// ADC Configuration test
#[test]
fn test_adc_configuration() {
    let exp_val = exp_adc_cfg(ContinuousAll, Ct1052, Ct1052, Ct1052, AvgN1024);
    let mut spi = SpiMock::new(&[write_txn(0x01, exp_val)]);
    let mut ina = INA239::new(spi.clone());
    ina.set_adc_config(ContinuousAll, Ct1052, Ct1052, Ct1052, AvgN1024)
        .unwrap();
    spi.done();
}

// Shunt voltage test
#[test]
fn test_shunt_volt() {
    let mut spi = SpiMock::new(&[read_txn(0x04, &0x0020_u16.to_be_bytes())]);
    let mut ina = INA239::new(spi.clone());
    ina.shunt_voltage().unwrap();
    spi.done();
}

// Bus voltage test
#[test]
fn test_bus_volt() {
    let mut spi = SpiMock::new(&[read_txn(0x05, &0x0010_u16.to_be_bytes())]);
    let mut ina = INA239::new(spi.clone());
    ina.bus_voltage().unwrap();
    spi.done();
}

// Internal die temperature test
#[test]
fn test_die_temp() {
    let mut spi = SpiMock::new(&[read_txn(0x06, &0x0040_u16.to_be_bytes())]);
    let mut ina = INA239::new(spi.clone());
    ina.die_temperature().unwrap();
    spi.done();
}

// Current result test
#[test]
fn test_current() {
    let mut spi = SpiMock::new(&[read_txn(0x07, &0x0001_u16.to_be_bytes())]);
    let mut ina = INA239::new(spi.clone());
    ina.current().unwrap();
    spi.done();
}

// Power result test
#[test]
fn test_power() {
    let mut spi = SpiMock::new(&[read_txn(0x08, &u24_be_bytes(0x0020))]);
    let mut ina = INA239::new(spi.clone());
    ina.power().unwrap();
    spi.done();
}

// Test shunt overvoltage threshold
#[test]
fn test_shunt_overvolt() {
    // 0x7FFF (32727) * 1.25uv = 0.04 V
    let exp_val = exp_shunt_volt(0.04, true) as u16;
    let mut spi = SpiMock::new(&[write_txn(0x0C, exp_val)]);
    let mut ina = INA239::new(spi.clone());
    ina.set_shunt_overvoltage_th(0.04).unwrap();
    spi.done();
}

// Tests get shunt overvoltage threshold
#[test]
fn test_get_shunt_overvoltage() {
    let mut spi = SpiMock::new(&[
        read_txn(0x0C, &0x7FFF_u16.to_be_bytes()),
        read_txn(0x0C, &0x8001_u16.to_be_bytes()),
    ]);
    let mut ina = INA239::new(spi.clone());
    let p_val = ina.get_shunt_overvoltage_th().unwrap();
    let n_val = ina.get_shunt_overvoltage_th().unwrap();
    debug_assert!(p_val >= 0.0, "Expected positive value");
    debug_assert!(n_val < 0.0, "Expected negative value");
    spi.done();
}

// Test shunt undervoltage threshold
#[test]
fn test_shunt_undervolt() {
    // 0x8000 (-32728) * 1.25uv = -0.04 V
    let exp_val = exp_shunt_volt(-0.04, true) as u16;
    let mut spi = SpiMock::new(&[write_txn(0x0D, exp_val)]);
    let mut ina = INA239::new(spi.clone());
    ina.set_shunt_undervoltage_th(-0.04).unwrap();
    spi.done();
}

// Tests get shunt undervoltage threshold
#[test]
fn test_get_shunt_undervoltage() {
    let mut spi = SpiMock::new(&[
        read_txn(0x0D, &0x7FFE_u16.to_be_bytes()),
        read_txn(0x0D, &0x8000_u16.to_be_bytes()),
    ]);
    let mut ina = INA239::new(spi.clone());
    let p_val = ina.get_shunt_undervoltage_th().unwrap();
    let n_val = ina.get_shunt_undervoltage_th().unwrap();
    debug_assert!(p_val >= 0.0, "Expected positive value, got {}", p_val);
    debug_assert!(n_val < 0.0, "Expected negative value, got {}", n_val);
    spi.done();
}

// Test Bus overvoltage threshold
// Set method
#[test]
fn test_bus_overvolt() {
    // 0x7FFF (32727) * 3.125mv = 102.2 V
    let exp_val = (102.0 / 3.125e-3) as u16;
    let mut spi = SpiMock::new(&[write_txn(0x0E, exp_val)]);
    let mut ina = INA239::new(spi.clone());
    ina.set_bus_overvoltage_th(102.0).unwrap();
    spi.done();
}

// get method
#[test]
fn test_get_bus_overvoltage() {
    // 0x7FFF (Reserved bit = 0)
    // 0xFFFF (Reserved bit = 1)
    let mut spi = SpiMock::new(&[
        read_txn(0x0E, &0x7FFF_u16.to_be_bytes()),
        read_txn(0x0E, &0xFFFF_u16.to_be_bytes()),
    ]);

    let mut ina = INA239::new(spi.clone());

    let val_res_zero = ina.get_bus_overvoltage_th().unwrap();
    let val_res_one = ina.get_bus_overvoltage_th().unwrap();

    // Assert that the reserved bit (MSB) was masked off and read as '0' only.
    assert_eq!(val_res_zero, val_res_one, "Reserved bit not zero");
    assert!(val_res_zero >= 0.0);

    spi.done(); //
}

// Test bus undervoltage threshold
// Set method
#[test]
fn test_bus_undervolt() {
    // 0x1 * 3.125mv = 0.003 V
    let exp_val = (0.003 / 3.125e-3) as u16;
    let mut spi = SpiMock::new(&[write_txn(0x0F, exp_val)]);
    let mut ina = INA239::new(spi.clone());
    ina.set_bus_undervoltage_th(0.003).unwrap();
    spi.done();
}

// get method
#[test]
fn test_get_bus_undervoltage() {
    // 0x0000 (Reserved bit = 0)
    // 0x8000 (Reserved bit = 1)
    let mut spi = SpiMock::new(&[
        read_txn(0x0F, &0x0000_u16.to_be_bytes()),
        read_txn(0x0F, &0x8000_u16.to_be_bytes()),
    ]);

    let mut ina = INA239::new(spi.clone());

    let val_res_zero = ina.get_bus_undervoltage_th().unwrap();
    let val_res_one = ina.get_bus_undervoltage_th().unwrap();

    // Assert that the reserved bit (MSB) was masked off and read as '0' only.
    assert_eq!(val_res_zero, val_res_one, "Reserved bit not zero");
    assert!(val_res_zero >= 0.0);

    spi.done(); //
}

// Tests temperature over-limit threshold
// Set method
#[test]
fn test_temp_overlimit() {
    // -0x800 (-2048) * 125mC = -256
    let neg_val = exp_temp_th(-256.1) as u16;

    // 0x7FF (2047) * 125mC = 255.8
    let pos_val = exp_temp_th(255.8) as u16;

    let expectations = [write_txn(0x10, neg_val), write_txn(0x10, pos_val)];
    let mut spi = SpiMock::new(&expectations);
    let mut ina = INA239::new(spi.clone());
    ina.set_temperature_limit(-256.1).unwrap();
    ina.set_temperature_limit(255.8).unwrap();
    spi.done();
}

// Get method
#[test]
fn test_get_temp_overlimit() {
    let mut spi = SpiMock::new(&[
        read_txn(0x10, &0x7FF0_u16.to_be_bytes()),
        read_txn(0x10, &0x8000_u16.to_be_bytes()),
    ]);
    let mut ina = INA239::new(spi.clone());

    let p_val = ina.get_temperature_limit().unwrap();
    let n_val = ina.get_temperature_limit().unwrap();

    debug_assert!(p_val >= 0.0, "Expected positive value, got {}", p_val);
    debug_assert!(n_val < 0.0, "Expected negative value, got {}", n_val);

    // Check temp overlimit value equal to max value = 255.x (approx.)
    debug_assert!((p_val - 255.0) <= 1.0, "Wrong value");
    // Check temp overlimit value equal to min value = -256.x (approx.)
    debug_assert!((n_val - 255.0) <= -1.0, "Wrong value");

    spi.done();
}

// Tests reserved bits always reads 0
#[test]
fn test_get_temp_overlimit_reserved_zero() {
    // 0x7FF0 (Reserved bit[3:0] = 0)
    // 0x7FFF (Reserved bit[3:0] = 1)
    let mut spi = SpiMock::new(&[
        read_txn(0x10, &0x7FF0_u16.to_be_bytes()),
        read_txn(0x10, &0x7FFF_u16.to_be_bytes()),
    ]);

    let mut ina = INA239::new(spi.clone());

    let val_res_zero = ina.get_temperature_limit().unwrap();
    let val_res_one = ina.get_temperature_limit().unwrap();

    // Assert that the lower nibble (reserved bits [3-0]) was read 0h.
    assert_eq!(val_res_zero, val_res_one, "Reserved bits not zero");

    spi.done(); //
}

// Tests Power over-limit threshold
// Set power over-limit
#[test]
fn test_power_overlimit() {
    let shunt_cal = exp_shunt_cal(10.0, 0.01, false);
    let current_lsb = 10.0 / 32_768.0;
    let power_lsb = 0.2 * current_lsb;

    // 0xFFFF (65535) * 256 * 0.2 * curr_lsb = 1023.
    let exp_val = (1000.0 / (256.0 * power_lsb)) as u16;

    let mut spi = SpiMock::new(&[write_txn(0x02, shunt_cal), write_txn(0x11, exp_val)]);
    let mut ina = INA239::new(spi.clone());
    ina.set_shunt_calibrate(10.0, 0.01).unwrap();
    ina.set_power_limit(1000.0).unwrap();
    spi.done();
}

// Get power over-limit
#[test]
fn test_get_power_overlimit() {
    let mut spi = SpiMock::new(&[read_txn(0x11, &0xFFFF_u16.to_be_bytes())]);
    let mut ina = INA239::new(spi.clone());
    let val = ina.get_power_limit().unwrap();
    debug_assert!(val >= 0.0, "Expected positive value");
    // Check power value equal to max value = 1023 (approx.)
    debug_assert!((val - 1023.0) <= 1.0, "Wrong value");
    spi.done();
}

// Test Diagnostic Flags read
#[test]
fn test_diag_flags() {
    let mut spi = SpiMock::new(&[read_txn(0xB, &0xFFFF_u16.to_be_bytes())]);
    let mut ina = INA239::new(spi.clone());
    let val = ina.diagnostic_flags().unwrap();
    // Check reserved bits b[11-10, 8] are zero
    debug_assert_eq!(val & 0x0D00, 0, "Reserved bits are not zero");
    spi.done();
}

// Test alert pin configuration
#[test]
fn test_alert_config() {
    let mut spi = SpiMock::new(&[
        read_txn(0xB, &0x0FFF_u16.to_be_bytes()),
        write_txn(0xB, 0xE2FF),
    ]);
    let mut ina = INA239::new(spi.clone());
    ina.config_alerts(AlertConfig {
        alert_latch: true,
        conversion_ready: true,
        slow_alert: true,
        alert_polarity: false,
    })
    .unwrap();
    spi.done();
}

// ---- Helper functions ----
fn read_txn(reg: u8, bytes: &[u8]) -> SpiTransaction<u8> {
    // Command byte for INA239 Read: Bit 7 = 1 (Read), Address B6-B1: Register address shifted left 1
    let cmd = 0x80 | ((reg & 0x3F) << 1);
    let expt_send = vec![cmd, 0x00, 0x00];
    let mock_resp = vec![0x00, bytes[0], bytes[1]];

    SpiTransaction::transfer_in_place(expt_send, mock_resp)
}

fn write_txn(reg: u8, value: u16) -> SpiTransaction<u8> {
    // Command byte for INA239 Write: Bit 7 = 0 (write), Address B6-B1: Register address shifted left 1
    let cmd = (reg & 0x3F) << 1;
    let bytes = value.to_be_bytes();
    let expt_send = vec![cmd, bytes[0], bytes[1]];

    SpiTransaction::write_vec(expt_send)
}

fn u24_be_bytes(value: u32) -> [u8; 3] {
    [(value >> 16) as u8, (value >> 8) as u8, value as u8]
}

//Expected shunt constant (SHUNT_CAL) calculation
fn exp_shunt_cal(max_current: f32, shunt_resistance: f32, adc_range: bool) -> u16 {
    let current_lsb = max_current / (1 << 15) as f32;
    let mut shunt_val = 819.2e6 * current_lsb * shunt_resistance;
    if adc_range {
        shunt_val *= 4.0;
    }
    shunt_val as u16
}

// Expected config register value calculation
fn exp_confg(conv_delay_ms: u16, adc_range: bool) -> u16 {
    let mut cfg_val = (conv_delay_ms >> 1) << 6;
    if adc_range {
        cfg_val |= 1 << 4;
    }
    cfg_val as u16
}

// Expected ADC config register value calculation
fn exp_adc_cfg(
    mode: Mode,
    busvolt_ct: Time,
    shuntvolt_ct: Time,
    tempe_ct: Time,
    adc_avgcount: Count,
) -> u16 {
    (mode as u16) << 12
        | (busvolt_ct as u16) << 9
        | (shuntvolt_ct as u16) << 6
        | (tempe_ct as u16) << 3
        | adc_avgcount as u16
}

// Shunt voltage calculations
fn exp_shunt_volt(volt: f32, adc_range: bool) -> i16 {
    let mut shunt_volt = volt / 5e-6;
    if adc_range {
        shunt_volt *= 4.0;
    }
    shunt_volt as i16
}

// Temperature limit expected value
fn exp_temp_th(temp_c: f32) -> i16 {
    ((temp_c / 125e-3) as i16) << 4
}
