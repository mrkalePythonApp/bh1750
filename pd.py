# -*- coding: utf-8 -*-
"""This file is part of the libsigrokdecode project.

Copyright (C) 2019 Libor Gabaj <libor.gabaj@gmail.com>

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, see <http://www.gnu.org/licenses/>.

"""

import sigrokdecode as srd


###############################################################################
# Enumeration classes
###############################################################################
class Address:
    """Enumeration of possible slave addresses.

    - Device address is determined by a pin, which the sensor's ADD0 pin is
      physically connected to.
    """

    (GND, VCC) = (0x23, 0x5c)


class Register:
    """Enumeration of possible slave register addresses."""

    (
        PWRDOWN, PWRUP, RESET, MTHIGH, MTLOW,  # Operation commands
        MCHIGH, MCHIGH2, MCLOW,     # Continuous measurement modes
        MOHIGH, MOHIGH2, MOLOW,     # One-time measurement modes
    ) = (
            0x00, 0x01, 0x07, 0x40, 0x60,
            0x10, 0x11, 0x13,
            0x20, 0x21, 0x23,
    )


class CommonBits:
    """Enumeration of common bits."""

    (RESERVED,) = (0xff,)


class MTregHighBits:
    """Range of high data bits of the measurement time register."""

    (MIN, MAX) = (0, 2)


class MTregLowBits:
    """Range of low data bits of the measurement time register."""

    (MIN, MAX) = (0, 4)


class AnnAddrs:
    """Enumeration of annotations for addresses."""

    (GND, VCC) = range(0, 2)


class AnnRegs:
    """Enumeration of annotations for registers."""

    (
        PWRDOWN, PWRUP, RESET, MTHIGH, MTLOW,
        MCHIGH, MCHIGH2, MCLOW,
        MOHIGH, MOHIGH2, MOLOW,
        DATA,
    ) = range(AnnAddrs.VCC + 1, (AnnAddrs.VCC + 1) + 12)


class AnnBits:
    """Enumeration of annotations for configuration bits."""

    (RESERVED, DATA) = range(AnnRegs.DATA + 1, (AnnRegs.DATA + 1) + 2)


class AnnInfo:
    """Enumeration of annotations for various information."""

    (
        WARN, BADADD, CHECK, WRITE, READ,
        LIGHT, MTREG, MTIME,
    ) = range(AnnBits.DATA + 1, (AnnBits.DATA + 1) + 8)


###############################################################################
# Parameters mapping
###############################################################################
addr_annots = {  # Convert value to annotation index
    Address.GND: AnnAddrs.GND,
    Address.VCC: AnnAddrs.VCC,
}

reg_annots = {  # Convert device register value to annotation index
    Register.PWRDOWN: AnnRegs.PWRDOWN,
    Register.PWRUP: AnnRegs.PWRUP,
    Register.RESET: AnnRegs.RESET,
    Register.MTHIGH: AnnRegs.MTHIGH,
    Register.MTLOW: AnnRegs.MTLOW,
    Register.MCHIGH: AnnRegs.MCHIGH,
    Register.MCHIGH2: AnnRegs.MCHIGH2,
    Register.MCLOW: AnnRegs.MCLOW,
    Register.MOHIGH: AnnRegs.MOHIGH,
    Register.MOHIGH2: AnnRegs.MOHIGH2,
    Register.MOLOW: AnnRegs.MOLOW,
}

radixes = {  # Convert radix option to format mask
    "Hex": "{:#02x}",
    "Dec": "{:#d}",
    "Oct": "{:#o}",
}

params = {
    "MTREG_TYP": 69,
    "ACCURACY_TYP": 1.20,    # Count per lux
    "ACCURACY_MAX": 1.44,
    "ACCURACY_MIN": 0.96,
    "UNIT_LIGHT": "lx",
}

###############################################################################
# Parameters anotations definitions
###############################################################################
"""
- If a parameter has a value, the last item of an annotation list is used
  repeatedly without a value.
- If a parameter has measurement unit alongside with value, the last two items
  are used repeatedly without that measurement unit.
"""
addresses = {
    AnnAddrs.GND: ["ADDR grounded", "ADDR_GND", "AG"],
    AnnAddrs.VCC: ["ADDR powered", "ADDR_VCC", "AV"],
}

registers = {
    AnnRegs.PWRDOWN: ["Power down", "Pwr Dwn", "Off", "D"],
    AnnRegs.PWRUP: ["Power up", "Pwr Up", "On", "U"],
    AnnRegs.RESET: ["Reset light register", "Reset light", "Reset",
                    "Rst", "R"],
    AnnRegs.MTHIGH: ["Measurement time high bits", "Mtime Hbits", "MTH", "H"],
    AnnRegs.MTLOW: ["Measurement time low bits", "Mtime Lbits", "MTL", "L"],
    AnnRegs.MCHIGH: ["Continuous measurement high resolution",
                     "Continuous high res", "Cont high", "CH"],
    AnnRegs.MCHIGH2: ["Continuous measurement double high resolution",
                      "Continuous double high res", "Cont double", "CH2"],
    AnnRegs.MCLOW: ["Continuous measurement low resolution",
                    "Continuous low res", "Cont low", "CL"],
    AnnRegs.MOHIGH: ["One time measurement high resolution",
                     "One time high res", "One high", "OH"],
    AnnRegs.MOHIGH2: ["One time measurement double high resolution",
                      "One time double high res", "One double", "OH2"],
    AnnRegs.MOLOW: ["One time measurement low resolution",
                    "One time low res", "One low", "OL"],
    AnnRegs.DATA: ["Illuminance data register", "Illuminance register",
                   "Illuminance", "Light", "L"],
}

bits = {
    AnnBits.RESERVED: ["Reserved bit", "Reserved", "Rsvd", "R"],
    AnnBits.DATA: ["MT bit", "MT", "M"],
}

info = {
    AnnInfo.WARN: ["Warnings", "Warn", "W"],
    AnnInfo.BADADD: ["Uknown slave address", "Unknown address", "Uknown",
                        "Unk", "U"],
    AnnInfo.CHECK: ["Slave presence check", "Slave check", "Check",
                       "Chk", "C"],
    AnnInfo.WRITE: ["Write", "Wr", "W"],
    AnnInfo.READ: ["Read", "Rd", "R"],
    AnnInfo.LIGHT: ["Ambient light", "Light", "L"],
    AnnInfo.MTREG: ["Measurement time register", "MTreg", "R"],
    AnnInfo.MTIME: ["Measurement time", "MTime", "MT", "T"],
}

def create_annots():
    annots = []
    # Address
    for attr, value in vars(AnnAddrs).items():
        if not attr.startswith('__'):
            annots.append(tuple(["addr-" + attr.lower(), addresses[value][0]]))
    # Register
    for attr, value in vars(AnnRegs).items():
        if not attr.startswith('__'):
            annots.append(tuple(["reg-" + attr.lower(), registers[value][0]]))
    # Bits
    for attr, value in vars(AnnBits).items():
        if not attr.startswith('__'):
            annots.append(tuple(["bit-" + attr.lower(), bits[value][0]]))
    # Info
    for attr, value in vars(AnnInfo).items():
        if not attr.startswith('__'):
            annots.append(tuple(["info-" + attr.lower(), info[value][0]]))
    return tuple(annots)


###############################################################################
# Decoder
###############################################################################
class Decoder(srd.Decoder):
    """Protocol decoder for digital temperature sensor ``TMP102``."""

    api_version = 3
    id = "bh1750"
    name = "BH1750"
    longname = "Digial ambient light sensor BH1750"
    desc = "Digital 16bit Serial Output Type Ambient Light Sensor IC, v 1.0.0."
    license = "gplv2+"
    inputs = ["i2c"]
    outputs = ["bh1750"]

    options = (
        {"id": "radix", "desc": "Numbers format", "default": "Hex",
         "values": ("Hex", "Dec", "Oct")},
        {"id": "params", "desc": "Datasheet parameter used",
         "default": "Typical",
         "values": ("Typical", "Maximal", "Minimal")},
    )

    create_annots()
    annotations = create_annots()
    annotation_rows = (
        ("bits", "Bits", (AnnBits.RESERVED, AnnBits.DATA)),
        ("regs", "Registers",
            tuple(range(AnnAddrs.GND, AnnRegs.DATA + 1))),
        ("info", "Info",
            tuple(range(AnnInfo.CHECK, AnnInfo.MTIME + 1))),
        ("warnings", "Warnings", (AnnInfo.WARN, AnnInfo.BADADD,)),
    )

    def __init__(self):
        """Initialize decoder."""
        self.reset()

    def reset(self):
        """Reset decoder and initialize instance variables."""
        # Common parameters for I2C sampling
        self.ss = 0         # Start sample
        self.es = 0         # End sample
        self.ssb = 0        # Start sample of an annotation transmission block
        self.ssd = 0        # Start sample of an annotation data block
        self.esd = 0        # End sample of an annotation data block
        self.bits = []      # List of recent processed byte bits
        self.bytes = []     # List of recent processed bytes
        self.write = None   # Flag about recent write action
        self.state = "IDLE"
        # Specific parameters for a device
        self.addr = Address.GND             # Slave address
        self.reg = Register.PWRDOWN         # Processed register
        self.mode = Register.MCHIGH         # Measurement mode
        self.mtreg = params["MTREG_TYP"]    # MTreg default value

    def start(self):
        """Actions before the beginning of the decoding."""
        self.out_ann = self.register(srd.OUTPUT_ANN)

    def compose_annot(self, ann_label, ann_value=None, ann_unit=None,
                      ann_action=None):
        """Compose list of annotations enriched with value and unit.

        Arguments
        ---------
        ann_label : list
            List of annotation label for enriching with values and units and
            prefixed with actions.
            *The argument is mandatory and has no default value.*
        ann_value : list
            List of values to be added item by item to all annotations.
        ann_unit : list
            List of measurement units to be added item by item to all
            annotations. The method does not add separation space between
            the value and the unit.
        ann_action : list
            List of action prefixes prepend item by item to all annotations.
            The method separates action and annotation with a space.

        Returns
        -------
        list of str
            List of a annotations potentially enriched with values and units
            with items sorted by length descending.

        Notes
        -----
        - Usually just one value and one unit is used. However for flexibility
          more of them can be used.
        - If the annotation values list is not defined, the annotation units
          list is not used, even if it is defined.

        """
        if not isinstance(ann_label, list):
            tmp = ann_label
            ann_label = []
            ann_label.append(tmp)

        if ann_value is None:
            ann_value = []
        elif not isinstance(ann_value, list):
            tmp = ann_value
            ann_value = []
            ann_value.append(tmp)

        if ann_unit is None:
            ann_unit = []
        elif not isinstance(ann_unit, list):
            tmp = ann_unit
            ann_unit = []
            ann_unit.append(tmp)

        if ann_action is None:
            ann_action = []
        elif not isinstance(ann_action, list):
            tmp = ann_action
            ann_action = []
            ann_action.append(tmp)
        if len(ann_action) == 0:
            ann_action = [""]

        # Compose annotation
        annots = []
        for act in ann_action:
            for lbl in ann_label:
                ann = "{} {}".format(act, lbl).strip()
                ann_item = None
                for val in ann_value:
                    ann_item = "{}: {}".format(ann, val)
                    annots.append(ann_item)  # Without units
                    for unit in ann_unit:
                        ann_item += "{}".format(unit)
                        annots.append(ann_item)  # With units
                if ann_item is None:
                    annots.append(ann)

        # Add last 2 annotation items without values
        if len(ann_value) > 0:
            for ann in ann_label[-2:]:
                annots.append(ann)
        annots.sort(key=len, reverse=True)
        return annots

    def put_data(self, bit_start, bit_stop, data):
        """Span data output across bit range.

        - Output is an annotation block from the start sample of the first bit
          to the end sample of the last bit.
        """
        self.put(self.bits[bit_start][1], self.bits[bit_stop][2],
                 self.out_ann, data)

    def put_bit_data(self, bit_reserved):
        """Span output under general data bit.

        - Output is an annotation block from the start to the end sample
          of a data bit.
        """
        annots = self.compose_annot(bits[AnnBits.DATA])
        self.put(self.bits[bit_reserved][1], self.bits[bit_reserved][2],
                 self.out_ann, [AnnBits.DATA, annots])

    def put_bit_reserve(self, bit_reserved):
        """Span output under reserved bit.

        - Output is an annotation block from the start to the end sample
          of a reserved bit.
        """
        annots = self.compose_annot(bits[AnnBits.RESERVED])
        self.put(self.bits[bit_reserved][1], self.bits[bit_reserved][2],
                 self.out_ann, [AnnBits.RESERVED, annots])

    def check_addr(self, addr_slave):
        """Check correct slave address."""
        if addr_slave in (Address.GND, Address.VCC):
            return True
        annots = self.compose_annot(AnnInfo.BADADD,
                                    "{:#04x}".format(self.addr))
        self.put(self.ssb, self.es, self.out_ann, [AnnInfo.BADADD, annots])
        return False

    def calculate_sensitivity(self):
        """Calculate measurement light sensitivity in lux per count."""
        suffix = self.options["params"][0:3].upper()
        resolution = params["ACCURACY_" + suffix] * params["MTREG_TYP"] / self.mtreg  # count/lux
        sensitivity = 1 / resolution  # lux/count
        if self.mode in [Register.MCHIGH2, Register.MOHIGH2]:
            sensitivity /= 2
        return sensitivity

    def calculate_light(self, rawdata):
        """Calculate ambient light.

        Arguments
        ---------
        rawdata : int
            Content of the illuminance data register.

        Returns
        -------
        float
            Ambient light in lux.

        """
        light = rawdata * self.calculate_sensitivity()
        return light

    def collect_data(self, databyte):
        """Collect data byte to a data cache."""
        self.esd = self.es
        if len(self.bytes) == 0:
            self.ssd = self.ss
            self.bytes.append(databyte)
        else:
            self.bytes.insert(0, databyte)

    def clear_data(self):
        """Clear data cache."""
        self.ssd = self.esd = 0
        self.bytes = []
        self.bits = []

    def handle_addr(self):
        """Process slave address."""
        if len(self.bytes) == 0:
            return
        # Registers row
        self.addr = self.bytes[0]
        ann_idx = addr_annots[self.addr]
        annots = self.compose_annot(addresses[ann_idx])
        self.put(self.ssd, self.esd, self.out_ann, [ann_idx, annots])
        self.clear_data()

    def handle_reg(self):
        """Process slave register and call its handler."""
        if len(self.bytes) == 0 or not self.write:
            return
        self.reg = self.bytes[0]
        # Handle measurement time registers
        mask_mthigh = ~((1 << (MTregHighBits.MAX + 1)) - 1)
        if (self.reg & mask_mthigh) == Register.MTHIGH:
            self.handle_mtreg_high()
            return
        mask_mtlow = ~((1 << (MTregLowBits.MAX + 1)) - 1)
        if (self.reg & mask_mtlow) == Register.MTLOW:
            self.handle_mtreg_low()
            return
        # Detect measurement mode registers
        if self.reg in range(Register.MCHIGH, Register.MOLOW + 1):
            self.mode = self.reg
        # Registers row
        ann_idx = reg_annots[self.reg]
        annots = self.compose_annot(registers[ann_idx])
        self.put(self.ssd, self.esd, self.out_ann, [ann_idx, annots])
        self.clear_data()

    def handle_mtreg_high(self):
        """Process measurement time register with high bits."""
        mask = (1 << (MTregLowBits.MAX + 1)) - 1
        self.mtreg &= mask  # Clear high bits
        mtreg = (self.reg << (MTregLowBits.MAX + 1)) & 0xff
        self.mtreg |= mtreg
        self.reg = Register.MTHIGH
        # Bits row - high bits
        for i in range(MTregHighBits.MIN, MTregHighBits.MAX + 1):
            self.put_bit_data(i)
        # Registers row
        annots = self.compose_annot(registers[AnnRegs.MTHIGH])
        self.put(self.ssd, self.esd, self.out_ann, [AnnRegs.MTHIGH, annots])
        self.clear_data()

    def handle_mtreg_low(self):
        """Process measurement time register with low bits."""
        mask = (1 << (MTregLowBits.MAX + 1)) - 1
        self.mtreg &= ~mask  # Clear low bits
        mtreg = self.reg & mask
        self.mtreg |= mtreg
        self.reg = Register.MTLOW
        # Bits row - low bits
        for i in range(MTregLowBits.MIN, MTregLowBits.MAX + 1):
            self.put_bit_data(i)
        # Registers row
        annots = self.compose_annot(registers[AnnRegs.MTLOW])
        self.put(self.ssd, self.esd, self.out_ann, [AnnRegs.MTLOW, annots])
        self.clear_data()

    def handle_data(self):
        """Process read data."""
        if self.write:
            # Info row
            if self.reg in [Register.MTHIGH, Register.MTLOW]:
                mtreg = radixes[self.options["radix"]].format(self.mtreg)
                unit = " ({:.2f} {}/cnt)".format(self.calculate_sensitivity(),
                                         params["UNIT_LIGHT"])
                annots = self.compose_annot(
                    info[AnnInfo.MTREG],
                    ann_action=info[AnnInfo.WRITE],
                    ann_value=mtreg,
                    ann_unit=unit,
                )
                self.put(self.ssb, self.es, self.out_ann, [AnnInfo.MTREG,
                                                           annots])
        else:
            regword = (self.bytes[1] << 8) + self.bytes[0]
            # Registers row
            annots = self.compose_annot(registers[AnnRegs.DATA])
            self.put(self.ssd, self.esd, self.out_ann, [AnnRegs.DATA, annots])
            # # Info row
            light = "{:.2f}".format(self.calculate_light(regword))
            unit = " {}".format(params["UNIT_LIGHT"])
            annots = self.compose_annot(
                info[AnnInfo.LIGHT],
                ann_action=info[AnnInfo.READ],
                ann_value=light,
                ann_unit=unit,
            )
            self.put(self.ssb, self.es, self.out_ann, [AnnInfo.LIGHT,
                                                       annots])
        self.clear_data()

    def decode(self, startsample, endsample, data):
        """Decode samples provided by parent decoder."""
        cmd, databyte = data
        self.ss, self.es = startsample, endsample
        # print(self.state, cmd)

        if cmd == "BITS":
            """Collect packet of bits that belongs to the following command.
            - Packet is in the form of list of bit lists:
                ["BITS", bitlist]
            - Bit list is a list of 3 items list
                [[bitvalue, startsample, endsample], ...]
            - Samples are counted for aquisition sampling frequency.
            - Parent decoder ``i2c``stores individual bits in the list from
              the least significant bit (LSB) to the most significant bit
              (MSB) as it is at representing numbers in computers, although I2C
              bus transmits data in oposite order with MSB first.
            """
            self.bits = databyte + self.bits
            return

        # State machine
        if self.state == "IDLE":
            """Wait for new I2C transmission"""
            # if cmd not in ["START", "START REPEAT"]:
            if cmd != "START":
                return
            self.ssb = self.ss
            self.state = "ADDRESS SLAVE"

        elif self.state == "ADDRESS SLAVE":
            """Wait for slave address"""
            if cmd in ["ADDRESS WRITE", "ADDRESS READ"]:
                if self.check_addr(databyte):
                    self.collect_data(databyte)
                    self.handle_addr()
                    if cmd == "ADDRESS READ":
                        self.write = False
                    elif cmd == "ADDRESS WRITE":
                        self.write = True
                    self.state = "REGISTER ADDRESS"
                else:
                    self.state = "IDLE"

        elif self.state == "REGISTER ADDRESS":
            """Process slave register"""
            if cmd in ["DATA WRITE", "DATA READ"]:
                self.collect_data(databyte)
                self.handle_reg()
                self.state = "REGISTER DATA"
            elif cmd in ["STOP", "START REPEAT"]:
                """Wait another transmittion."""
                self.state = "IDLE"

        elif self.state == "REGISTER DATA":
            """Process data of a slave register.
            - Individual command or data can end either with repeated start
              condition or with stop condition.
            """
            if cmd in ["DATA WRITE", "DATA READ"]:
                self.collect_data(databyte)
            elif cmd == "START REPEAT":
                """Output read data and continue in transmission."""
                self.handle_data()
                self.ssb = self.ss
                self.state = "ADDRESS SLAVE"
            elif cmd == "STOP":
                """Output read data and wait another transmission."""
                self.handle_data()
                self.state = "IDLE"
