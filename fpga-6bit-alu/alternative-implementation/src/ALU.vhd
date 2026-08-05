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

    component full_adder is
        Port (
            a, b, Cin : in  STD_LOGIC;
            sum, cout : out STD_LOGIC
        );
    end component;

    -- internal signals
    signal sumresult, subresult, andresult, orresult, xorresult, norresult : STD_LOGIC_VECTOR(5 downto 0);
    signal b_sub : STD_LOGIC_VECTOR(5 downto 0);
    signal carry_add, carry_sub : STD_LOGIC_VECTOR(0 to 6);
    signal Cout_add, Cout_sub, Ofl_add, Ofl_sub : STD_LOGIC;
    signal res_internal : STD_LOGIC_VECTOR(5 downto 0);  -- Fixed: new internal signal

    -- enable signals
    signal enable_add, enable_sub, enable_and, enable_or, enable_xor, enable_nor : STD_LOGIC;

begin
    -- enable decoding
    enable_add <= NOT aluop(2) AND NOT aluop(1) AND NOT aluop(0); -- 000
    enable_sub <= NOT aluop(2) AND aluop(1) AND NOT aluop(0);      -- 010
    enable_and <= aluop(2) AND NOT aluop(1) AND NOT aluop(0);      -- 100
    enable_or  <= aluop(2) AND NOT aluop(1) AND aluop(0);          -- 101
    enable_xor <= aluop(2) AND aluop(1) AND NOT aluop(0);          -- 110
    enable_nor <= aluop(2) AND aluop(1) AND aluop(0);              -- 111

    -- logic operations
    andresult <= a AND b;
    orresult  <= a OR b;
    xorresult <= a XOR b;
    norresult <= NOT (a OR b);

    -- sub: invert b for twos complement
    b_sub <= NOT b;

    -- initial carryins
    carry_add(0) <= '0';
    carry_sub(0) <= '1';

    -- ripple carry add
    gen_add: for i in 0 to 5 generate
        adder_inst: full_adder port map(
            a    => a(i),
            b    => b(i),
            Cin  => carry_add(i),
            sum  => sumresult(i),
            cout => carry_add(i+1)
        );
    end generate;
    Cout_add <= carry_add(6);

    -- ripple carry sub
    gen_sub: for i in 0 to 5 generate
        sub_inst: full_adder port map(
            a    => a(i),
            b    => b_sub(i),
            Cin  => carry_sub(i),
            sum  => subresult(i),
            cout => carry_sub(i+1)
        );
    end generate;
    Cout_sub <= carry_sub(6);

    -- output with enable gating
    gen_res: for i in 0 to 5 generate
    begin
        res_internal(i) <=
            (sumresult(i) AND enable_add) OR
            (subresult(i) AND enable_sub) OR
            (andresult(i) AND enable_and) OR
            (orresult(i)  AND enable_or) OR
            (xorresult(i) AND enable_xor) OR
            (norresult(i) AND enable_nor);
    end generate;

    -- assign the internal result to the output port
    res <= res_internal;

    -- zero flag
    zero <= NOT (res_internal(0) OR res_internal(1) OR res_internal(2) OR res_internal(3) OR res_internal(4) OR res_internal(5));

    -- carry out flag
    cout <= (Cout_add AND enable_add) OR (Cout_sub AND enable_sub);

    -- overflow flag
    Ofl_add <= (a(5) XOR b(5)) AND (a(5) XOR sumresult(5));
    Ofl_sub <= (a(5) XOR b_sub(5)) AND (a(5) XOR subresult(5));
    ofl <= (Ofl_add AND enable_add) OR (Ofl_sub AND enable_sub);

end gate_level;
