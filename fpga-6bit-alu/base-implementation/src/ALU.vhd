library IEEE;
use IEEE.STD_LOGIC_1164.ALL;

entity ALU is
    Port (
        a, b    : in  STD_LOGIC_VECTOR(5 downto 0);
        aluop   : in  STD_LOGIC_VECTOR(2 downto 0);
        res     : out STD_LOGIC_VECTOR(5 downto 0);
        zero    : out STD_LOGIC;
        cout    : out STD_LOGIC;
        ofl     : out STD_LOGIC
    );
end ALU;

architecture gate_level of ALU is

    -- full adder component
    component full_adder is
        Port (
            a, b, Cin : in  STD_LOGIC;
            sum, cout : out STD_LOGIC
        );
    end component;

    -- 8to1 mux component
    component mux8to1 is
        Port (
            I0, I1, I2, I3, I4, I5, I6, I7 : in  STD_LOGIC_VECTOR(5 downto 0);
            S : in  STD_LOGIC_VECTOR(2 downto 0);
            output : out STD_LOGIC_VECTOR(5 downto 0)
        );
    end component;

    -- internal signals
    signal alu_result, sumresult, subresult, andresult, orresult, xorresult, norresult : STD_LOGIC_VECTOR(5 downto 0);
    signal b_sub : STD_LOGIC_VECTOR(5 downto 0);
    signal carry_add, carry_sub : STD_LOGIC_VECTOR(0 to 6);
    signal Cout_add, Cout_sub, Ofl_add, Ofl_sub : STD_LOGIC;

    -- aliases to avoid i+1
    alias carry_add_now  : STD_LOGIC_VECTOR(0 to 5) is carry_add(0 to 5);
    alias carry_add_next : STD_LOGIC_VECTOR(0 to 5) is carry_add(1 to 6);
    alias carry_sub_now  : STD_LOGIC_VECTOR(0 to 5) is carry_sub(0 to 5);
    alias carry_sub_next : STD_LOGIC_VECTOR(0 to 5) is carry_sub(1 to 6);

begin

    -- logic operations
    andresult <= a AND b;
    orresult  <= a OR b;
    xorresult <= a XOR b;
    norresult <= NOT (a OR b);

    -- subtraction B inversion
    b_sub <= NOT b;

    -- set carryin values
    carry_add(0) <= '0';
    carry_sub(0) <= '1';

    -- ripple carry adder: add
    gen_add: for i in 0 to 5 generate
        adder_inst: full_adder port map(
            a    => a(i),
            b    => b(i),
            Cin  => carry_add_now(i),
            sum  => sumresult(i),
            cout => carry_add_next(i)
        );
    end generate;
    Cout_add <= carry_add(6);

    -- ripple carry adder: sub
    
    gen_sub: for i in 0 to 5 generate
        sub_inst: full_adder port map(
            a    => a(i),
            b    => b_sub(i),
            Cin  => carry_sub_now(i),
            sum  => subresult(i),
            cout => carry_sub_next(i)
        );
    end generate;
    Cout_sub <= carry_sub(6);

    -- mux for result
    MUX8: mux8to1 port map(
        I0 => sumresult,
        I1 => (others => '0'),
        I2 => subresult,
        I3 => (others => '0'),
        I4 => andresult,
        I5 => orresult,
        I6 => xorresult,
        I7 => norresult,
        S  => aluop,
        output => alu_result
    );

    -- final alu output
    res <= alu_result;

    -- zero flag
    zero <= NOT (alu_result(0) OR alu_result(1) OR alu_result(2) OR alu_result(3) OR alu_result(4) OR alu_result(5));

    -- carry out for add and sub
    cout <= (Cout_add AND (NOT aluop(2) AND NOT aluop(1) AND NOT aluop(0))) OR
            (Cout_sub AND (NOT aluop(2) AND aluop(1) AND NOT aluop(0)));

    -- overflow
    Ofl_add <= (a(5) XOR b(5)) AND (a(5) XOR sumresult(5));
    Ofl_sub <= (a(5) XOR b_sub(5)) AND (a(5) XOR subresult(5));

    ofl <= (Ofl_add AND (NOT aluop(2) AND NOT aluop(1) AND NOT aluop(0))) OR
           (Ofl_sub AND (NOT aluop(2) AND aluop(1) AND NOT aluop(0)));

end gate_level;
