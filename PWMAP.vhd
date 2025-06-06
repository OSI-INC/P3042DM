-- VHDL netlist generated by SCUBA Diamond (64-bit) 3.12.1.454
-- Module  Version: 2.8
--C:\lscc\diamond\3.12\ispfpga\bin\nt64\scuba.exe -w -n PWMAP -lang vhdl -synth synplify -bus_exp 7 -bb -arch xo2c00 -type rom -addr_width 8 -num_rows 256 -data_width 8 -outdata UNREGISTERED -memfile c:/kevan/a3042/p3042dm/pwmap.mem -memformat hex 

-- Mon Apr 14 14:52:20 2025

library IEEE;
use IEEE.std_logic_1164.all;
-- synopsys translate_off
library MACHXO2;
use MACHXO2.components.all;
-- synopsys translate_on

entity PWMAP is
    port (
        Address: in  std_logic_vector(7 downto 0); 
        Q: out  std_logic_vector(7 downto 0));
end PWMAP;

architecture Structure of PWMAP is

    -- local component declarations
    component ROM256X1A
        generic (INITVAL : in std_logic_vector(255 downto 0));
        port (AD7: in  std_logic; AD6: in  std_logic; AD5: in  std_logic; 
            AD4: in  std_logic; AD3: in  std_logic; AD2: in  std_logic; 
            AD1: in  std_logic; AD0: in  std_logic; DO0: out  std_logic);
    end component;
    attribute NGD_DRC_MASK : integer;
    attribute NGD_DRC_MASK of Structure : architecture is 1;

begin
    -- component instantiation statements
    mem_0_7: ROM256X1A
        generic map (initval=> X"FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFC000000000000000000000000000")
        port map (AD7=>Address(7), AD6=>Address(6), AD5=>Address(5), 
            AD4=>Address(4), AD3=>Address(3), AD2=>Address(2), 
            AD1=>Address(1), AD0=>Address(0), DO0=>Q(7));

    mem_0_6: ROM256X1A
        generic map (initval=> X"FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF800003FFFFC0000000000000000000000")
        port map (AD7=>Address(7), AD6=>Address(6), AD5=>Address(5), 
            AD4=>Address(4), AD3=>Address(3), AD2=>Address(2), 
            AD1=>Address(1), AD0=>Address(0), DO0=>Q(6));

    mem_0_5: ROM256X1A
        generic map (initval=> X"FFFFFFFFFFFFFFFFFFFFFFFFFFFFE007FE003FF003FF00000000000000000000")
        port map (AD7=>Address(7), AD6=>Address(6), AD5=>Address(5), 
            AD4=>Address(4), AD3=>Address(3), AD2=>Address(2), 
            AD1=>Address(1), AD0=>Address(0), DO0=>Q(5));

    mem_0_4: ROM256X1A
        generic map (initval=> X"FFFFFFFFFFFFFFFFFFFFFFFFFFFC1F07C1F03E0F83E0F8000000000000000000")
        port map (AD7=>Address(7), AD6=>Address(6), AD5=>Address(5), 
            AD4=>Address(4), AD3=>Address(3), AD2=>Address(2), 
            AD1=>Address(1), AD0=>Address(0), DO0=>Q(4));

    mem_0_3: ROM256X1A
        generic map (initval=> X"FFFFFFFFFFFFFFFFFFFFFFFFFFF39CE739CE318C6318C6000000000000000000")
        port map (AD7=>Address(7), AD6=>Address(6), AD5=>Address(5), 
            AD4=>Address(4), AD3=>Address(3), AD2=>Address(2), 
            AD1=>Address(1), AD0=>Address(0), DO0=>Q(3));

    mem_0_2: ROM256X1A
        generic map (initval=> X"FFFFFFFFFFFFFFFFFFFFFFFFFFEB5AD6B5AD294A5294A5000000000000000000")
        port map (AD7=>Address(7), AD6=>Address(6), AD5=>Address(5), 
            AD4=>Address(4), AD3=>Address(3), AD2=>Address(2), 
            AD1=>Address(1), AD0=>Address(0), DO0=>Q(2));

    mem_0_1: ROM256X1A
        generic map (initval=> X"FFFFFFFFFFFFFFFFFFFFFFFFFFC6739CE7398C6318C673800000000000000000")
        port map (AD7=>Address(7), AD6=>Address(6), AD5=>Address(5), 
            AD4=>Address(4), AD3=>Address(3), AD2=>Address(2), 
            AD1=>Address(1), AD0=>Address(0), DO0=>Q(1));

    mem_0_0: ROM256X1A
        generic map (initval=> X"FFFFFFFFFFFFFFFFFFFFFFFFFFDA94A56B5A94A56B5A94800000000000000000")
        port map (AD7=>Address(7), AD6=>Address(6), AD5=>Address(5), 
            AD4=>Address(4), AD3=>Address(3), AD2=>Address(2), 
            AD1=>Address(1), AD0=>Address(0), DO0=>Q(0));

end Structure;

-- synopsys translate_off
library MACHXO2;
configuration Structure_CON of PWMAP is
    for Structure
        for all:ROM256X1A use entity MACHXO2.ROM256X1A(V); end for;
    end for;
end Structure_CON;

-- synopsys translate_on
