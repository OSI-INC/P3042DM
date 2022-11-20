-- <pre> Radio-Frequency Detector Module (A3042DM) Firmware, Toplevel Unit

-- Compile with Synplify Pro. The message decoder fails if compiled with the Lattice synthesizer.

-- [18-NOV-22] Copy A3038DM v8.1, make file names generic. Tag as A3042DM v1.1, could be used in
-- the A3038DM.

-- [18-NOV-22] 

-- Global Constantslibrary ieee;  
library ieee;  
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity main is 
	port (
		Q : in std_logic; -- Comparator Output from Detector
		SDO : in std_logic;  -- Serial Data Out from ADC
		SCK : out std_logic; -- Serial Clock for ADC
		NCS : out std_logic; -- Negated Chip Select for ADC
		DSD_in : in std_logic; -- Data Strobe, Downstream
		dbd : out std_logic_vector(7 downto 0); -- Data Bus, Downstream
		DSU : out std_logic; -- Data Strobe, Upstream
		dbu : in std_logic_vector(7 downto 0); -- Data Bus, Upstream
		DMRST : in std_logic; -- Detector Module Reset (DC0)
		DMRC_in : in std_logic; -- Detector Module Read Control (DC1)
		DMERR : out std_logic; -- Detector Module Error (DC2)
		MRDY : out std_logic; -- Message Ready (DC3)
		SHOW : in std_logic; -- Show All Lamps (DC4)
		DMCK : in std_logic; -- Detector Module Clock (DC7)
		LED : out std_logic_vector(5 downto 1); -- Indictors Lamps
		TP : out std_logic_vector(4 downto 1) -- Test Points
	);
	
	constant power_calibration : integer := 0;
	constant run_led_num : integer := 1;
	constant err_led_num : integer := 2;
	constant inc_led_num : integer := 3;
	constant rcv_led_num : integer := 4;
	constant pwr_led_num : integer := 5;
end;

architecture behavior of main is

-- Attributes to guide the compiler.
	attribute syn_keep : boolean;
	attribute nomerge : string;

-- Detector Control Bus Signals
	signal ERROR : std_logic; -- Detector module error, to master.
	signal DSD, DMRC : std_logic; -- Synchronized inputs.

-- Message Decoder Signals
	signal CONTINUE : std_logic; -- Tell message detector to continue detection.
	signal RSTDCR : std_logic; -- Reset the decoder.
	signal INCOMING : std_logic; -- The message detector is receiving a message.
	signal RECEIVED : std_logic; -- The message detector has received a complete message.
	signal message_id : integer range 0 to 255; -- id of received message.
	signal message_data : std_logic_vector(15 downto 0); -- contents of received message.

-- Clock and Timing Signals.
	signal CK : std_logic; -- Decoder Clock 40 MHz
	signal IOCK : std_logic; -- Input-Output Clock 20 MHz
	signal SLWCK : std_logic; -- Slow Clock 10 kHz
	signal LCK : std_logic; -- Lamp Clock 10 Hz
	signal LOCK : std_logic; -- PLL Lock
	signal clock_divA, clock_divB : std_logic_vector(7 downto 0);
	
-- ADC Signals
	signal pwr, pwr_rcv, pwr_intensity : std_logic_vector(7 downto 0);
	signal PRDY : boolean;		
-- Functions and Procedures	
	function to_std_logic (v: boolean) return std_ulogic is
	begin if v then return('1'); else return('0'); end if; end function;
	
-- Message FIFO
	signal received_record, recalled_record : std_logic_vector(31 downto 0);
	signal FIFO_EMPTY, FIFO_FULL, WRMSG, RDMSG : std_logic;
	signal FIFO_EMPTY_ERR, FIFO_FULL_ERR : boolean;

begin
	-- The message decoder runs off 40 MHz. We receive 8 MHz on DMCK and use 
	-- a PLL to multiply to 40 MHz.
	Clock : entity PLL port map (
		CLKI => DMCK,
		CLKOP => CK,
		LOCK => LOCK
	);

	-- Synchronize control inputs with respect to the rising edge of CK. Generate 
	-- IOCK at 20 MHz for use with the serial readout of the power measurement
	-- analog to digital converter (ADC).
	Synchronizer : process (CK) is
	constant max_state : integer := 15;
	begin
		-- Generate 20-MHz Input-Output Clock.
		if rising_edge(CK) then
			IOCK <= to_std_logic(IOCK = '0');
		end if;
		
		-- Synchronize inputs asserted by controller.
		if rising_edge(CK) then
			DSD <= DSD_in;
			DMRC <= DMRC_in;
		end if;
	end process;
	
	-- We use the lamp clock to provide human-visible signalling with lamps.
	-- Our divider runs of CK, which is 40 MHz. We divide CK by 256 twice to 
	-- get 610 Hz. We divide by 61 to get 10 Hz to control lamp signals. We 
	-- divide CK by 256 once to get 156 kHz, then divide by 16 to get 10 kHz.
	Slow_Clocks : process (CK) is
	variable div1,div2 : integer range 0 to 63;
	begin
		if rising_edge(CK) then
			clock_divA <= std_logic_vector(unsigned(clock_divA)+1);
		end if;
		if rising_edge(clock_divA(7)) then
			clock_divB <= std_logic_vector(unsigned(clock_divB) + 1);
			if div1 = 15 then
				div1 := 0;
			else
				div1 := div1 + 1;
			end if;
			SLWCK <= to_std_logic(div1 < 9);
		end if;
		if rising_edge(clock_divB(7)) then
			if div2 = 60 then
				div2 := 0;
			else
				div2 := div2 + 1;
			end if;
			LCK <= to_std_logic(div2 < 30);
		end if;
	end process;
	
	-- The Message Decoder watches the incoming logic sequence and looks for
	-- messages. When it sees an incoming message, it asserts INCOMING. When
	-- it sees a complete message, it asserts RECEIVED. It provides the message
	-- id and data for storage, and waits until it receives RST before
	-- clearing its id and data values and looking for the next message. If
	-- it receives RST at any other time, it returns to its reast state, waiting
	-- for a new message to arrive.
	Decoder : entity message_decoder port map (
		CK => CK,-- Clock, 40.000 MHz
		Q => Q, -- Demodulator Output
		RST => RSTDCR, -- Reset the Decoder
		INCOMING => INCOMING, -- Message Incoming
		RECEIVED => RECEIVED, -- Message Received
		message_id => message_id, -- Eight-Bit Message ID
		message_data => message_data -- Sixteen-Bit Message Data
	);
		
	-- The Detector Control process operates the various detector control
	-- lines that are shared by all detector modules and the master. In
	-- particular, there are three global shared lines that each detector
	-- can assert or leave tri-state. These are the global error, incoming,
	-- and received lines.
	Detector_Control : process (CK) is
	begin
		if rising_edge(CK) then
			if ERROR = '1' then
				DMERR <= '1';
			else
				DMERR <= 'Z';
			end if;
			
			if RECEIVED = '1' then
				MRDY <= '1';
			else
				MRDY <= 'Z';
			end if;
		end if;
	end process;
	
	-- The Message FIFO receives new 32-bit message records and allows the
	-- them to be read out later by the detector module daisy chain. The FIFO
	-- writes occur on the falling edge of CK. We asser WrEn on the prior
	-- rising edge, which allows 12.5 ns (half of 40-MHz period) for the FIFO
	-- to prepare for WrClock. The logic chip requires 11.4 ns setup. On the
	-- read, we set up RdEn on the falling edge and read on the rising edge
	-- of CK, which again allows 12.5 ns setup where 11.7 ns is required.
	Message_FIFO : entity FIFO port map (
		Data => received_record,
        WrClock => CK,
        RdClock => CK,
        WrEn => WRMSG,
        RdEn => RDMSG,
        Reset => DMRST,
        RPReset => DMRST,
        Q => recalled_record,
        Empty => FIFO_EMPTY,
        Full => FIFO_FULL
	);	
	
	-- We sample the power detector output and read out the ADC when we
	-- see INCOMING asserted. We drive the ADC chip select input low 
	-- (asserting !CS). We then wait until MRDY is asserted, at which time 
	-- it is safe to read out the ADC without disturbing message reception. 
	-- We generate SCK at 20 MHz and read out three dummy bits, eight data 
	-- bits, and three terminating bits. The ADC transitions to input tracking 
	-- after the third terminating bit. Once the readout is complete, we wait 
	-- for INCOMING to be unasserted. The PRDY flag indicates that a new power 
	-- measurement is ready, and will be asserted until the power readout 
	-- returns to rest.
	Power_Readout : process (IOCK) is
	constant pwr_end : integer := 31;
	constant wait_end : integer := 37;
	variable state, next_state : integer range 0 to 63;
	begin
		if DMRST = '1' then
			state := 0;
			SCK <= '1';
			NCS <= '1';
			pwr <= "00000000";
			PRDY <= false;
		elsif rising_edge(IOCK) then
			next_state := state;
			if state = 0 then
				if (INCOMING = '1') then 
					next_state := 1;
				else
					next_state := 0;
				end if;
			elsif state = 1 then
				if (INCOMING = '0') then
					next_state := 0;
				elsif (RECEIVED = '1') then
					next_state := 2;
				else
					next_state := 1;
				end if;
			elsif state = pwr_end then
				if (INCOMING = '0') then
					next_state := state + 1;
				else
					next_state := pwr_end;
				end if;
			elsif state = wait_end then
				next_state := 0;
			else
				next_state := state + 1;
			end if;
			case state is
				when 0  => SCK <= '1'; NCS <= '1';  -- Idle
				when 1  => SCK <= '1'; NCS <= '0';  -- Acquire input voltage
				when 2  => SCK <= '0'; NCS <= '0';  -- Initiate conversion
				when 3  => SCK <= '1'; NCS <= '0';
				when 4  => SCK <= '0'; NCS <= '0';  -- Continue conversion
				when 5  => SCK <= '1'; NCS <= '0';
				when 6  => SCK <= '0'; NCS <= '0';  -- Complete conversion
				when 7  => SCK <= '1'; NCS <= '0';
				when 8  => SCK <= '0'; NCS <= '0';  -- Bit 7
				when 9  => SCK <= '1'; NCS <= '0';
				when 10 => SCK <= '0'; NCS <= '0';  -- Bit 6
				when 11 => SCK <= '1'; NCS <= '0';
				when 12 => SCK <= '0'; NCS <= '0';  -- Bit 5
				when 13 => SCK <= '1'; NCS <= '0';
				when 14 => SCK <= '0'; NCS <= '0';  -- Bit 4
				when 15 => SCK <= '1'; NCS <= '0';
				when 16 => SCK <= '0'; NCS <= '0';  -- Bit 3
				when 17 => SCK <= '1'; NCS <= '0';
				when 18 => SCK <= '0'; NCS <= '0';  -- Bit 2
				when 19 => SCK <= '1'; NCS <= '0';
				when 20 => SCK <= '0'; NCS <= '0';  -- Bit 1
				when 21 => SCK <= '1'; NCS <= '0';
				when 22 => SCK <= '0'; NCS <= '0';  -- Bit 0
				when 23 => SCK <= '1'; NCS <= '0';
				when 24 => SCK <= '0'; NCS <= '0';  -- Set up next cycle
				when 25 => SCK <= '1'; NCS <= '0';
				when 26 => SCK <= '0'; NCS <= '0';  -- Set up next cycle
				when 27 => SCK <= '1'; NCS <= '0';
				when 28 => SCK <= '0'; NCS <= '0';  -- Track input
				when 29 => SCK <= '1'; NCS <= '0';
				when 30 => SCK <= '1'; NCS <= '1';  -- Terminate cycle
				when others => SCK <= '1'; NCS <= '1'; -- Idle
			end case;
			case state is
				when 9 | 11 | 13 | 15 | 17 | 19 | 21 | 23 => 
					pwr(7 downto 1) <= pwr(6 downto 0);
					pwr(0) <= SDO;
				when others =>
					pwr <= pwr;
			end case;
			PRDY <= (state >= pwr_end);
			state := next_state;
		end if;
	end process;
	
	-- The Message Recorder saves 32-bit message records in the FIFO. The
	-- top eight bits are the message id, the next sixteen are the message
	-- data, and the final eight are the power.
	Message_Recorder : process (CK,DMRST) is
	variable state, next_state : integer range 0 to 3;
	begin
		if DMRST = '1' then
			state := 0;
			RSTDCR <= '1';
			pwr_rcv <= "00000000";
		elsif rising_edge(CK) then
			next_state := state;
			RSTDCR <= '0';
			WRMSG <= '0';
			case state is
				when 0 =>
					if (RECEIVED = '1') and PRDY then 
						next_state := 1;
					end if;
				when 1 =>
					next_state := 2;
					WRMSG <= '1';
					pwr_rcv <= pwr;
				when 2 =>
					RSTDCR <= '1';
					if RECEIVED = '0' then
						next_state := 3;
					else
						next_state := 2;
					end if;
				when 3 =>
					RSTDCR <= '1';
					next_state := 0;
			end case;
			state := next_state;
		end if;
		
		received_record(31 downto 24) <= std_logic_vector(to_unsigned(message_id,8));
		received_record(23 downto 8) <= message_data;
		received_record(7 downto 0) <= pwr;
	end process;
	
	-- The Message Reader asserts MRDY when the message buffer contains a message ready to
	-- be read, and it responds to all daisy chain message readout cycles, either by providing
	-- a message from this detector module, or by allowing a message from another detector
	-- module to pass through. When the daisy chain master reads the detector module daisy
	-- chain position, the Message Reader takes part in the incrementing of the upstream
	-- date that generates an automatic determination of the index of the source of all
	-- messages. 
	Message_Reader : process (CK,DMRST) is
	variable state, next_state : integer range 0 to 15;
	variable local_read : boolean;
	begin
	
		-- On reset, we return to our rest state, we do not read out a message from
		-- the message buffer, and we drive zeros onto the downstream data lines.
		-- We set our local_read flag to false, so we will be blocking the incoming
		-- Data Strobe Downstream (DSD) from passing through to Data Strobe Upstream
		-- (DSU).
		if (DMRST = '1') then
			state := 0;
			RDMSG <= '0';
			dbd <= (others => '0');
			local_read := true;
			
		-- Our readout state machine runs off CK and remains in its rest state, blocking
		-- the data strobe, so long as Detector Module Read Control (DMRC) is unasserted. 
		-- When DRC is asserted and we have a message waiting, we read it out and block 
		-- DSD from DSU. We proceed with the five-byte readout of identifier, high data 
		-- byte, low data byte, power, and daisy chain index in response to data strobes. 
		-- If DMRC is unasserted any time during the readout, the Message Reader returns
		-- to its rest state and the message is abandoned. When DMRC is asserted and we
		-- do not have a message waiting, we do not read a message from the buffer, but
		-- instead relay DSD to DSU and Data Bus Upstream (dbu) to Data Bus Downstrem
		-- (dbd). We watch the data strobes, and after the completion of the fourth,
		-- we increment dbu on its way to dbd. The detector module being read will provide
		-- a zero, which is incremented by each detector module it passes through, so
		-- that the byte arriving at the daisy chain master is equal to the source
		-- module's daisy chain index, the first detector in the chain being index zero.		
		elsif rising_edge(CK) then
			next_state := state;
			RDMSG <= '0';
			
			-- We make our MRDY synchronous with our clock to avoid any glitches on
			-- the global line. We drive it high, but never drive it low, because it
			-- is shared among all detector modules on the daisy chain.
			if (FIFO_EMPTY = '0') then
				MRDY <= '1';
			else
				MRDY <= 'Z';
			end if;
			
			-- The rest stae when DMRC is unasserted, we block DSD from DSU.
			if (DMRC = '0') then
				next_state := 0;
				local_read := true;
			end if;
			
			-- The readout state machine. We later use the state to configure
			-- dbd and DSU.
			if (DMRC = '1') then
				case state is 
					when 0 => 
						next_state := 1;
						local_read := (FIFO_EMPTY = '0');
					when 1 => 
						next_state := 2;
						if local_read then RDMSG <= '1'; end if;
					when 2 => if (DSD = '1') then next_state := 3; end if;
					when 3 => if (DSD = '0') then next_state := 4; end if;
					when 4 => if (DSD = '1') then next_state := 5; end if;
					when 5 => if (DSD = '0') then next_state := 6; end if;
					when 6 => if (DSD = '1') then next_state := 7; end if;
					when 7 => if (DSD = '0') then next_state := 8; end if;
					when 8 => if (DSD = '1') then next_state := 9; end if;
					when 9 => if (DSD = '0') then next_state := 10; end if;
					when 10 => if (DSD = '1') then next_state := 11; end if;
					when 11 => if (DSD = '0') then next_state := 12; end if;
					when others => next_state := state;
				end case;
			end if;
			
			state := next_state;
		end if;
		
		-- We control DSU and dbd with combinatorial logic so that we can minimize
		-- the propagation delay up and down the daisy chain. The detector module
		-- introduces one pin-to-pin delay, which for our ZE chips is around 7 ns,
		-- rather than the 25-ns period of our clock.
		if local_read then
			DSU <= '0';
			case state is
				when 0 | 1 => dbd <= (others => '0');
				when 2 | 3 => dbd <= recalled_record(31 downto 24);
				when 4 | 5 => dbd <= recalled_record(23 downto 16);
				when 6 | 7 => dbd <= recalled_record(15 downto 8);
				when 8 | 9 => dbd <= recalled_record(7 downto 0);
				when others => dbd <= (others =>'0');				
			end case;
		else 
			DSU <= DSD;
			case state is
				when 0 | 1 => dbd <= (others => '0');
				when 2 | 3 => dbd <= dbu;
				when 4 | 5 => dbd <= dbu;
				when 6 | 7 => dbd <= dbu;
				when 8 | 9 => dbd <= dbu;
				when others => dbd <= std_logic_vector(to_unsigned(to_integer(unsigned(dbu))+1,8));		
			end case;
		end if;
		
	end process;
	
	-- The error detector looks for overflow or empty FIFO conflicts.
	Error_Detection : process (CK,DMRST) is
	begin
		if DMRST = '1' then
			FIFO_FULL_ERR <= false;
			FIFO_EMPTY_ERR <= false;
		elsif rising_edge(CK) then
			if (WRMSG = '1') and (FIFO_FULL = '1') then
				FIFO_FULL_ERR <= true;
			end if;
			if (RDMSG = '1') and (FIFO_EMPTY = '1') then
				FIFO_EMPTY_ERR <= true;
			end if;
		end if;
		
		ERROR <= to_std_logic(FIFO_EMPTY_ERR or FIFO_FULL_ERR or (LOCK = '0'));
	end process;
	
	-- The run indicator glows steady if power is on, logic is programmed,
	-- and the clocks are running.
	Run_Indicator : process (SLWCK) is
	variable counter : integer range 0 to 15;
	variable run_led_on : boolean;
	begin
		if DMRST = '1' then
			counter := 0;
			run_led_on := false;
		elsif rising_edge(SLWCK) then
			counter := counter +1;
		end if;
		
		run_led_on := (counter rem 4) = 3;
			
		if SHOW = '1' then
			run_led_on := true;
		end if;
		
		LED(run_led_num) <= to_std_logic(run_led_on);
	end process;
	
	-- The status indicator is off when all is well, and flashes 
	-- to indicate error conditions.
	Status_Indicator : process (LCK) is
	variable counter : integer range 0 to 15;	variable err_led_on : boolean;
	begin
		if rising_edge(LCK) then
			counter := counter + 1;
			err_led_on := false;
		end if;
		
		err_led_on := 
			(FIFO_FULL_ERR and (counter >= 0) and (counter <= 1))
			or (FIFO_EMPTY_ERR and (counter >= 4) and (counter <= 11))
			or (LOCK = '0');
		
		if SHOW = '1' then
			err_led_on := true;
		end if;
		
		LED(err_led_num) <= to_std_logic(err_led_on);
	end process;
	
	-- Indoming LED indicates message is being decoded now.
	Incoming_Indicator : process (CK) is 
	variable counter : integer range 0 to 65535;
	constant pulse_length : integer := 32768;
	begin
		if rising_edge(CK) then
			if SHOW = '1' then
				LED(inc_led_num) <= '1';
			elsif (counter = 0) then
				if (INCOMING = '1') then
					counter := 1;
				else
					counter := 0;
				end if;
				LED(inc_led_num) <='0';	
			else	
				if (counter = pulse_length) then
					counter := 0;
				else
					counter := counter + 1;
				end if;
				LED(inc_led_num) <= '1';				
			end if;
		end if;
	end process;
	
	-- Received LED indicates complete message received.
	Received_Indicator : process (CK) is 
	variable counter : integer range 0 to 65535;
	constant pulse_length : integer := 32768;
	begin
		if rising_edge(CK) then
			if SHOW = '1' then
				LED(rcv_led_num) <= '1';
			elsif (counter = 0) then
				if (RECEIVED = '1') then
					counter := 1;
				else
					counter := 0;
				end if;
				LED(rcv_led_num) <= '0';
			else
				if (counter = pulse_length) then
					counter := 0;
				else
					counter := counter + 1;
				end if;
				LED(rcv_led_num) <= '1';				
			end if;
		end if;
	end process;

	-- Power Indicator Look-Up Table. Maps detector power values to display
	-- intensity values.
	Power_Map : entity PWMAP port map (
		Address => pwr_rcv,
		Q => pwr_intensity
	);

	-- Power LED brightness proportional to the log of most recetnly-received 
	-- message's power. We use the slow clock to generate a lamp signal with
	-- duty cycle proportional to log power.
	Power_Indicator : process (SLWCK) is 
	variable counter : integer range 0 to 255;
	begin
		if rising_edge(SLWCK) then
			if SHOW = '1' then
				LED(pwr_led_num) <= '1';
			elsif counter < to_integer(unsigned(pwr_intensity)) then
				LED(pwr_led_num) <= '1';	
			else
				LED(pwr_led_num) <= '0';
			end if;
			counter := counter + 1;
		end if;
	end process;
	
	TP(1) <= INCOMING;
	TP(2) <= RECEIVED;
	TP(3) <= DSD;
	TP(4) <= FIFO_EMPTY;
end behavior;