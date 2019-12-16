import smbus

ADDRESSES = [0x48, 0x49, 0x4A, 0x4B, 0x4C, 0x4D, 0x4E, 0x4F]
BUSNUMS = [0, 1]

class MCP3221(object):
	def __init__(self, address=0x48, busnum=0, refereceVoltage=3.3, voltageDivider=[10, 37]):
		if (address not in ADDRESSES):
			raise ValueError("Invalid Address: {0:#x}".format(address))
		if (busnum not in BUSNUMS):
			raise ValueError("Invalid bus number: {0:d}".format(busnum))
		self.address = address
		self.busnum = busnum
		self.refereceVoltage = refereceVoltage
		self.voltageDivider = voltageDivider

		self.bus = smbus.SMBus(self.busnum)
		self.readVoltage()

	def readVoltage(self):
		data = self.bus.read_i2c_block_data(self.address, 0x00, 2)
		res = (((data[0] << 8) + data[1])/4095)*self.refereceVoltage/self.voltageDivider[0]*self.voltageDivider[1]
		return res
