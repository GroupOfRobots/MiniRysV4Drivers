#!/usr/bin/env python3
import time
from mcp3221 import MCP3221

# Initialize TMP Object
mcp = MCP3221(0x48, 0, 3.3, [10, 37])

while True:
    print("Current voltage: {:.3f} V".format(mcp.readVoltage()))
    time.sleep(1)
