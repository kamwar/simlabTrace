# SDRAM initialization script for the AT91SAM7SE
#-----------------------------------------------
# Configure master clock
echo Configuring the master clock...\n
# Enable main oscillator
set *0xFFFFFC20 = 0x00004001
while ((*0xFFFFFC68 & 0x1) == 0)
end

# Enable PLL
set *0xFFFFFC2C = 0x1048100E
while ((*0xFFFFFC68 & 0x4) == 0)
end

# Select prescaler value
set *0xFFFFFC30 = 0x00000004
while ((*0xFFFFFC68 & 0x8) == 0)
end

# Select master clock
set *0xFFFFFC30 = 0x00000007
while ((*0xFFFFFC68 & 0x8) == 0)
end

echo Master clock ok.\n
echo Configuring the SDRAM controller...\n

# Enable EBI chip select for the SDRAM
set *0xFFFFFF80 = 0x2

# Enable EBI pios
# PMC
set *0xFFFFFC10 = 0x1C
# PIOA
set *0xFFFFF404 = 0x3F800000
set *0xFFFFF474 = 0x3F800000
# PIOB
set *0xFFFFF604 = 0x0003FFFF
set *0xFFFFF674 = 0x0003FFFF
# PIOC
set *0xFFFFF804 = 0x0000FFFF
set *0xFFFFF870 = 0x0000FFFF

# SDRAM configuration (see corresponding application note)
set *0xFFFFFFB8 = 0x21922159

set *0xFFFFFFB0 = 0x11
set *0x20000000 = 0

set *0xFFFFFFB0 = 0x12
set *0x20000000 = 0

set *0xFFFFFFB0 = 0x14
set *0x20000000 = 0
set *0xFFFFFFB0 = 0x14
set *0x20000000 = 0
set *0xFFFFFFB0 = 0x14
set *0x20000000 = 0
set *0xFFFFFFB0 = 0x14
set *0x20000000 = 0
set *0xFFFFFFB0 = 0x14
set *0x20000000 = 0
set *0xFFFFFFB0 = 0x14
set *0x20000000 = 0
set *0xFFFFFFB0 = 0x14
set *0x20000000 = 0
set *0xFFFFFFB0 = 0x14
set *0x20000000 = 0

set *0xFFFFFFB0 = 0x13
set *0x20000000 = 0

set *0xFFFFFFB0 = 0x10
set *0x20000000 = 0

set *0xFFFFFFB4 = 0x150

echo SDRAM configuration ok.\n