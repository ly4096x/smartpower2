# smartpower2

This version allows the SmartPower2 to connect to an existing hotspot. When it cannot connect to that hotspot within 10 seconds, it will become a hotspot by itself.

You can configure the hotspot to connect to in the `Configuration` web page. The `IP address` field is used for the fall-back hotspot, not for static IP allocation when connecting to another hotspot. When the SmartPower2 connects to a hotspot, the `IP address` field will show the DHCP assigned IP, instead of the fall-back hotspot IP. This is a known bug but since it's not affecting usage so there's no plan to fix it.
