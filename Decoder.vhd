-- <pre> Subcutaneous Transmitter (SCT) Message Decoder for Animal Location Tracker (ALT)

-- Receives as input the bit sequence, Q, from the discriminator of the A3038DM detector module. 
-- Interprets the sequence with the help of a 40-MHz clock, which allows it to synchronize its
-- state to that of the transmitter by using the eleven synchronizing bits with which each SCT
-- transmission begins. 

-- Version 1: [01-APR-21] Translated from ABEL program P302702A12_Decoder.abl.

-- Version 2: [10-MAY-21] Remove the CNT (continue) input. We will use the RST (reset)
-- input to restore the decoder to its initial state. In order to simplify the detector-- readout process, we reset all decoders after they store a message record in their 
-- message buffers. The decoder stays in its message-store state until reset. We correct
-- bugs in the reset implementation and forbid the reception of messages with ID exactly
-- divisible by sixteen.

library ieee;  
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity message_decoder is 
	port (
		CK : in std_logic; -- 40-MHz Clock
		Q : in std_logic; -- Demodulator Output
		RST : in std_logic; -- Reset
		INCOMING : out std_logic; -- Message Incoming
		RECEIVED : out std_logic; -- Message Received		
		message_id : out integer range 0 to 255;
		message_data : out std_logic_vector (15 downto 0)
	);
end;

architecture behavior of message_decoder is 

constant min_sync_edges : integer := 5;
constant num_message_bits : integer := 24;
constant min_inc_bits : integer := 16;

-- Global Signals
signal SQ, DSQ : boolean;

-- Functions and Procedures	
	function to_std_logic (v: boolean) return std_ulogic is
	begin if v then return('1'); else return('0'); end if; end function;

begin 

	synchronizer : process (CK) is
	begin
		if rising_edge (CK) then
			SQ <= (Q = '1');
			DSQ <= SQ;
		end if;
	end process;

	decoder : process (CK,RST) is 
	
	variable rvs, next_rvs : integer range 0 to 31 := 0;
	variable rvd : std_logic_vector (23 downto 0);
	variable channel_id, completion_code : integer range 0 to 15;
	variable ec : integer range 0 to 31;
	variable done, valid : boolean;
	
	constant rvs_rest : integer := 0;
	constant rvs_s1 : integer := 1;
	constant rvs_s2 : integer := 2;
	constant rvs_s3 : integer := 3;
	constant rvs_s4 : integer := 4;
	constant rvs_s5 : integer := 5;
	constant rvs_s6 : integer := 6;
	constant rvs_s7 : integer := 7;
	constant rvs_s8 : integer := 8;
	constant rvs_s9 : integer := 9;
	constant rvs_s10 : integer := 10;

	constant rvs_r2 : integer := 11;
	constant rvs_r3 : integer := 12;
	constant rvs_r4 : integer := 13;
	constant rvs_r5 : integer := 14;
	constant rvs_r6 : integer := 15;

	constant rvs_receive_start : integer := 16;
	constant rvs_receive_1 : integer := 17;
	constant rvs_receive_2 : integer := 18;
	constant rvs_receive_3 : integer := 19;
	constant rvs_receive_4 : integer := 20;
	
	constant rvs_one : integer := 24;
	constant rvs_zero : integer := 25;
	constant rvs_check : integer := 26;
	constant rvs_store : integer := 27;
	
	constant rvs_clr_ec : integer := 31;

	begin
		if (RST = '1') then
			rvs := rvs_rest;
			rvd := "000000000000000000000000";
			ec := 0;
			RECEIVED <= '0';
		elsif rising_edge(CK) then
			next_rvs := rvs_rest;
			RECEIVED <= '0';
			case rvs is 
				when rvs_rest => -- Waiting for a rising edge on Q.
					if SQ and (not DSQ) then 
						next_rvs := rvs_s1; 
					else
						next_rvs := rvs_rest;
					end if;
					rvd := "000000000000000000000000";
				when rvs_s1 => -- Check for !Q too soon.
					if not SQ then 
						next_rvs := rvs_clr_ec; 
					else
						next_rvs := rvs_s2;
					end if;
				when rvs_s2 => -- Check for !Q too soon.
					if not SQ then 
						next_rvs := rvs_clr_ec; 
					else
						next_rvs := rvs_s3;
					end if;
				when rvs_s3 => -- Check for !Q.
					if not SQ then 
						next_rvs := rvs_s6; 
					else
						next_rvs := rvs_s4;
					end if;
					if (ec < min_sync_edges) then
						ec := ec + 1;
					end if;
				when rvs_s4 => -- Check for !Q.
					if not SQ then 
						next_rvs := rvs_s6; 
					else
						next_rvs := rvs_s5;
					end if;
				when rvs_s5 => 
					-- If Q is still HI, and we have enough edges, this is the start bit.
					-- But if Q is still HI, and we do not have enough edges, we abort.
					-- If Q is LO, we are still receiving start bits.
					if SQ and (ec >= min_sync_edges) then 
						next_rvs := rvs_receive_start; 
					elsif SQ and (ec < min_sync_edges) then
						next_rvs := rvs_clr_ec;
					else
						next_rvs := rvs_s6;
					end if;
				when rvs_s6 => -- Check for Q too soon.
					if SQ then 
						next_rvs := rvs_clr_ec;
					else
						next_rvs := rvs_s7;
					end if;
				when rvs_s7 => -- Check for Q too soon.
					if SQ then 
						next_rvs := rvs_clr_ec;
					else
						next_rvs := rvs_s8;
					end if;
				when rvs_s8 => -- Check for Q.
					if SQ then 
						next_rvs := rvs_s1;
					else
						next_rvs := rvs_s9;
					end if;
				when rvs_s9 => -- Check for Q.
					if SQ then 
						next_rvs := rvs_s1;
					else
						next_rvs := rvs_s10;
					end if;
				when rvs_s10 => -- Check for Q, abort if not.
					if SQ then 
						next_rvs := rvs_s1;
					else
						next_rvs := rvs_clr_ec;
					end if;
					
				when rvs_receive_start =>
					if SQ and (not DSQ) then 
						next_rvs := rvs_one;
					elsif (not SQ) and DSQ then
						next_rvs := rvs_zero;
					else
						next_rvs := rvs_receive_2;
					end if;
					ec := 0;
					
				when rvs_r2 =>
					if (SQ /= DSQ) then
						next_rvs := rvs_clr_ec;
					else 
						next_rvs := rvs_r3;
					end if;
					ec := ec + 1;
				when rvs_r3 =>
					next_rvs := rvs_r4;
				when rvs_r4 =>
					next_rvs := rvs_r5;
				when rvs_r5 =>
					next_rvs := rvs_r6;
				when rvs_r6 =>
					next_rvs := rvs_receive_1;
					
				when rvs_receive_1 =>
					if SQ and (not DSQ) then
						next_rvs := rvs_one;
					elsif (not SQ) and DSQ then 
						next_rvs := rvs_zero;
					else 
						next_rvs := rvs_receive_2;
					end if;
				when rvs_receive_2 =>
					if SQ and (not DSQ) then
						next_rvs := rvs_one;
					elsif (not SQ) and DSQ then 
						next_rvs := rvs_zero;
					else 
						next_rvs := rvs_receive_3;
					end if;
				when rvs_receive_3 =>
					if SQ and (not DSQ) then
						next_rvs := rvs_one;
					elsif (not SQ) and DSQ then 
						next_rvs := rvs_zero;
					else 
						next_rvs := rvs_receive_4;
					end if;
				when rvs_receive_4 =>
					if SQ and (not DSQ) then
						next_rvs := rvs_one;
					elsif (not SQ) and DSQ then 
						next_rvs := rvs_zero;
					else 
						next_rvs := rvs_clr_ec;
					end if;
					
				when rvs_one =>
					if not SQ then
						next_rvs := rvs_clr_ec;
					elsif not DONE then
						next_rvs := rvs_r2;
					else
						next_rvs := rvs_check;
					end if;
					rvd(23 downto 1) := rvd(22 downto 0);
					rvd(0) := '1';
				when rvs_zero =>
					if SQ then 
						next_rvs := rvs_clr_ec;
					elsif not DONE then 
						next_rvs := rvs_r2;
					else
						next_rvs := rvs_check;
					end if;
					rvd(23 downto 1) := rvd(22 downto 0);
					rvd(0) := '0';
	
				when rvs_check =>
					if VALID then
						next_rvs := rvs_store;
					else
						next_rvs := rvs_clr_ec;
					end if;
					
				when rvs_store =>
					next_rvs := rvs_store;
					RECEIVED <= '1';
					
				when rvs_clr_ec =>
					next_rvs := rvs_rest;
					ec := 0;
					
				when others =>
					next_rvs := rvs_rest;
					ec := 0;
			end case;
			
			rvs := next_rvs;
		end if;
		
		DONE := (ec = num_message_bits);
		channel_id := to_integer(unsigned(rvd(23 downto 20)));
		completion_code := to_integer(unsigned(rvd(3 downto 0)));
		VALID := (channel_id /= 0);
		if (ec >= min_inc_bits) then
			INCOMING <= '1';
		else
			INCOMING <= '0';
		end if;
	
		message_data <= rvd(19 downto 4);
		message_id <= (16 * (completion_code - 15 + channel_id)) + channel_id;
	end process;
end behavior;

