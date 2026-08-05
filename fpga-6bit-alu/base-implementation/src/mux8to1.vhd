library IEEE;
use IEEE.STD_LOGIC_1164.ALL;

entity mux8to1 is
    Port (
        I0, I1, I2, I3, I4, I5, I6, I7 : in  STD_LOGIC_VECTOR(5 downto 0);
        S : in  STD_LOGIC_VECTOR(2 downto 0);
        output : out STD_LOGIC_VECTOR(5 downto 0)
    );
end mux8to1;

architecture gate_level of mux8to1 is
    signal sel0_n, sel1_n, sel2_n : STD_LOGIC;
    signal sel : STD_LOGIC_VECTOR(7 downto 0);
begin

    -- inverted selection lines
    sel0_n <= NOT S(0);
    sel1_n <= NOT S(1);
    sel2_n <= NOT S(2);

    -- selection logic
    sel(0) <= sel2_n AND sel1_n AND sel0_n; -- 000
    sel(1) <= sel2_n AND sel1_n AND S(0);   -- 001
    sel(2) <= sel2_n AND S(1) AND sel0_n;   -- 010
    sel(3) <= sel2_n AND S(1) AND S(0);     -- 011
    sel(4) <= S(2) AND sel1_n AND sel0_n;   -- 100
    sel(5) <= S(2) AND sel1_n AND S(0);     -- 101
    sel(6) <= S(2) AND S(1) AND sel0_n;     -- 110
    sel(7) <= S(2) AND S(1) AND S(0);       -- 111

    -- bitwise AND and OR logic to select correct input
    gen_output: for i in 0 to 5 generate
        output(i) <= 
            (I0(i) AND sel(0)) OR
            (I1(i) AND sel(1)) OR
            (I2(i) AND sel(2)) OR
            (I3(i) AND sel(3)) OR
            (I4(i) AND sel(4)) OR
            (I5(i) AND sel(5)) OR
            (I6(i) AND sel(6)) OR
            (I7(i) AND sel(7));
    end generate;

end gate_level;
