use crate::{hal, Channel, Error, Pca9685, Register};

impl<I2C, E> Pca9685<I2C>
where
    I2C: hal::blocking::i2c::Write<Error = E> + hal::blocking::i2c::WriteRead<Error = E>,
{
    /// Set double register to specific value not touching `full ON/OFF` flag.
    fn set_double_register_without_flag(&mut self, register: u8, value: u16) -> Result<(), Error<E>> {
        if value > 4095 {
            return Err(Error::InvalidInputData);
        }
        let reg_h = self.read_register(register + 1)?;
        let value = ((reg_h & 0x10) as u16) << 8 | value;
        self.write_double_register(register, value)
    }

    /// Set the `ON` counter for the selected channel.
    ///
    /// This method does not touch `full ON` flag.
    ///
    /// Note that the full off setting takes precedence over the `on` settings.
    /// See section 7.3.3 "LED output and PWM control" of the datasheet for
    /// further details.
    pub fn set_channel_on(&mut self, channel: Channel, value: u16) -> Result<(), Error<E>> {
        let reg = get_register_on(channel);
        self.set_double_register_without_flag(reg, value)
    }

    /// Set the `OFF` counter for the selected channel.
    ///
    /// This method does not touch `full OFF` flag.
    pub fn set_channel_off(&mut self, channel: Channel, value: u16) -> Result<(), Error<E>> {
        let reg = get_register_off(channel);
        self.set_double_register_without_flag(reg, value)
    }

    /// Set `full ON/OFF` flag on specific register
    fn set_register_full_flag(&mut self, register: u8, flag_value: bool) -> Result<(), Error<E>> {
        let register = register + 1; // flag is in high register
        let reg_h = self.read_register(register)?;
        if flag_value && (reg_h & 0x10 == 0) {
            self.i2c.write(self.address, &[register, reg_h | 0x10]).map_err(Error::I2C)
        } else if !flag_value && (reg_h & 0x10 != 0) {
            self.i2c.write(self.address, &[register, reg_h & 0x0f]).map_err(Error::I2C)
        } else {
            Ok(())
        }
    }

    /// Set the channel always on by setting `full ON` flag.
    ///
    /// Note that the full off setting takes precedence over the `on` settings.
    ///
    /// See section 7.3.3 "LED output and PWM control" of the datasheet for
    /// further details.
    pub fn set_channel_full_on(&mut self, channel: Channel, flag_value: bool) -> Result<(), Error<E>> {
        let reg = get_register_on(channel);
        self.set_register_full_flag(reg, flag_value)
    }

    /// Set the channel always off by setting `full OFF` flag.
    ///
    /// This takes precedence over the `on` settings.
    ///
    /// See section 7.3.3 "LED output and PWM control" of the datasheet for
    /// further details.
    pub fn set_channel_full_off(&mut self, channel: Channel, flag_value: bool) -> Result<(), Error<E>> {
        let reg = get_register_off(channel);
        self.set_register_full_flag(reg, flag_value)
    }

    /// Get the effective pulse length from `OFF` and `ON` counters.
    ///
    /// This takes into account `full ON/OFF` flags.
    pub fn get_effective_pulse(&mut self, channel: Channel) -> Result<u16, Error<E>> {
        let reg = get_register_on(channel);
        self.enable_auto_increment()?;

        let mut data = [0, 0, 0, 0];
        self.i2c.write_read(self.address, &[reg], &mut data).map_err(Error::I2C)?;

        // full off - highest priority
        if (data[3] & 0x10) != 0 {
            return Ok(0);
        }

        // full on
        if (data[1] & 0x10) != 0 {
            return Ok(4095);
        }

        // else normal mode
        let on_t = ((data[1] as u16) << 8) | data[0] as u16;
        let off_t = ((data[3] as u16) << 8) | data[2] as u16;

        if off_t >= on_t {
            Ok(off_t - on_t)
        } else {
            Ok(4095 - on_t + off_t)
        }
    }

}

macro_rules! get_register {
    ($channel:expr, $($C:ident, $reg:ident),*) => {
        match $channel {
            $(
                Channel::$C  => Register::$reg,
            )*
        }
    };
}

fn get_register_on(channel: Channel) -> u8 {
    get_register!(
        channel, C0, C0_ON_L, C1, C1_ON_L, C2, C2_ON_L, C3, C3_ON_L, C4, C4_ON_L, C5, C5_ON_L, C6,
        C6_ON_L, C7, C7_ON_L, C8, C8_ON_L, C9, C9_ON_L, C10, C10_ON_L, C11, C11_ON_L, C12,
        C12_ON_L, C13, C13_ON_L, C14, C14_ON_L, C15, C15_ON_L, All, ALL_C_ON_L
    )
}

fn get_register_off(channel: Channel) -> u8 {
    get_register!(
        channel,
        C0,
        C0_OFF_L,
        C1,
        C1_OFF_L,
        C2,
        C2_OFF_L,
        C3,
        C3_OFF_L,
        C4,
        C4_OFF_L,
        C5,
        C5_OFF_L,
        C6,
        C6_OFF_L,
        C7,
        C7_OFF_L,
        C8,
        C8_OFF_L,
        C9,
        C9_OFF_L,
        C10,
        C10_OFF_L,
        C11,
        C11_OFF_L,
        C12,
        C12_OFF_L,
        C13,
        C13_OFF_L,
        C14,
        C14_OFF_L,
        C15,
        C15_OFF_L,
        All,
        ALL_C_OFF_L
    )
}
