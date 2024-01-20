LIBRARY ieee;
USE ieee.std_logic_1164.all;

ENTITY tx IS
  GENERIC(
    mclk_sclk_ratio  :  INTEGER := 2; --serial clock ratio
    sclk_cs_ratio : INTEGER := 34; --chip select ratio
    d_width          :  INTEGER := 16);  --data width
  PORT(
    reset_n    :  IN   STD_LOGIC;                             --asynchronous active low reset
    mclk       :  IN   STD_LOGIC;                             --master clock
    sd_tx      :  OUT  STD_LOGIC;                             --serial data transmit
    cs         :  out STD_LOGIC;        
    sclk_dac   :  out STD_LOGIC;
    data_tx  :  IN   STD_LOGIC_VECTOR(d_width-1 DOWNTO 0));  --right channel data to transmit
END tx;

ARCHITECTURE logic OF tx IS

  SIGNAL sclk_int       :  STD_LOGIC := '0';                      --internal serial clock signal
  SIGNAL cs_int         :  STD_LOGIC := '1';                      --internal chip select signal
  SIGNAL data_tx_int  :  STD_LOGIC_VECTOR(d_width-1 DOWNTO 0);  --internal data buffer
BEGIN  
  
  PROCESS(mclk, reset_n)
    VARIABLE sclk_cnt  :  INTEGER := 0;  --counter of master clocks during half period of serial clock
    VARIABLE cs_cnt    :  INTEGER := 0;  --counter of serial clock toggles during half period of word select
  BEGIN
    
    IF(reset_n = '0') THEN                                           --asynchronous reset
      sclk_cnt := 0;                                                   --clear mclk/sclk counter
      cs_cnt := 0;                                                     --clear sclk/ws counter
      sclk_int <= '0';                                                 --clear serial clock signal
      cs_int <= '0';                                                   --clear word select signal
      data_tx_int <= (OTHERS => '0');                                --clear internal left channel tx data buffer
      sd_tx <= '0';                                                    --clear serial data transmit output
    ELSIF(mclk'EVENT AND mclk = '1') THEN                            --master clock rising edge
      IF(sclk_cnt < mclk_sclk_ratio/2-1) THEN                          --less than half period of sclk
        sclk_cnt := sclk_cnt + 1;                                        --increment mclk/sclk counter
      ELSE                                                             --half period of sclk
        sclk_cnt := 0;                                                   --reset mclk/sclk counter
        sclk_int <= NOT sclk_int;                                        --toggle serial clock
        IF(cs_cnt < sclk_cs_ratio-1) THEN                                --less than half period of ws
          IF(sclk_int = '1' AND cs_cnt < d_width*2) THEN 
            IF cs_int = '0' THEN
			 sd_tx <= data_tx_int(d_width-1);
			 data_tx_int <= data_tx_int(d_width-2 DOWNTO 0) & '0';
		    END IF;
          END IF;
        cs_cnt := cs_cnt + 1;    
        ELSE                                                            --half period of ws
          cs_cnt := 0;                                                    --reset sclk/ws counter
          cs_int <= NOT cs_int;                                           --toggle word select
          data_tx_int <= data_tx;                                     --latch in right channel data to transmit
        END IF;
      END IF;
    END IF;    
  END PROCESS;
  
  sclk_dac <= sclk_int;
  cs <= cs_int;
END logic;
    