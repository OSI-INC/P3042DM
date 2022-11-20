-- VHDL module instantiation generated by SCUBA Diamond (64-bit) 3.12.1.454
-- Module  Version: 5.8
-- Sat Nov 19 22:07:59 2022

-- parameterized module component declaration
component FIFO
    port (Data: in  std_logic_vector(31 downto 0); 
        WrClock: in  std_logic; RdClock: in  std_logic; 
        WrEn: in  std_logic; RdEn: in  std_logic; Reset: in  std_logic; 
        RPReset: in  std_logic; Q: out  std_logic_vector(31 downto 0); 
        Empty: out  std_logic; Full: out  std_logic);
end component;

-- parameterized module component instance
__ : FIFO
    port map (Data(31 downto 0)=>__, WrClock=>__, RdClock=>__, WrEn=>__, 
        RdEn=>__, Reset=>__, RPReset=>__, Q(31 downto 0)=>__, Empty=>__, 
        Full=>__);
