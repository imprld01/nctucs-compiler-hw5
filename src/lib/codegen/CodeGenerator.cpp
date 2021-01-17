#include "codegen/CodeGenerator.hpp"

#include <cassert>
#include <cstdarg>
#include <cstdio>

#include "visitor/AstNodeInclude.hpp"

typedef SymbolEntry::KindEnum Kind;
using std::string;

CodeGenerator::CodeGenerator(const char *in_file_name, const char *out_file_name, SymbolManager *symbol_manager)
    : in_file_name(in_file_name), symbol_manager(symbol_manager) {
    this->out_fp = fopen(out_file_name, "w");
    assert(this->out_fp != NULL && "fopen() fails");
}

CodeGenerator::~CodeGenerator() {
    fclose(this->out_fp);
}

void CodeGenerator::dumpInstrs(const char *format, ...) {
    va_list args;
    va_start(args, format);
    vfprintf(this->out_fp, format, args);
    va_end(args);
}

void CodeGenerator::dumpGloblVar(const string &varName, const PType &varType) {
    if (varType.isPrimitiveInteger()) {
        dumpInstrs(".comm %s, 4, 4\n", varName.c_str());
    }  //
    else {
        // TODO
    }
}

void CodeGenerator::dumpGloblConst(const string &varName, const Constant &cnst) {
    if (cnst.getTypePtr()->isPrimitiveInteger()) {
        dumpInstrs(".section    .rodata\n");
        dumpInstrs("    .align 2\n");
        dumpInstrs("    .globl %s\n", varName.c_str());
        dumpInstrs("    .type %s, @object\n", varName.c_str());
        dumpInstrs("%s:\n", varName.c_str());
        dumpInstrs("    .word %d\n", cnst.integer());
    }  //
    else {
        // TODO
    }
}

void CodeGenerator::dumpGlobl(const SymbolTable &table) {
    for (const auto &ptre : table.getEntries()) {
        const auto &e = *ptre;
        switch (e.getKind()) {
            case Kind::kConstantKind:
                dumpGloblConst(e.getName(), *e.getAttribute().constant());
                break;
            case Kind::kVariableKind:
                dumpGloblVar(e.getName(), *e.getTypePtr());
                break;
            default:
                // nothing to do
                break;
        }
    }
}

void CodeGenerator::dumpHeader() {
    dumpInstrs("    .file \"%s\"\n", in_file_name);
    dumpInstrs("    .option nopic\n");
}

void FuncModel::forLocal(SymbolEntry &e, int space) {
    stackpt -= space;
    e.stackLoc = stackpt;
}

FuncModel &CodeGenerator::funcModel() {
    return funcModels.back();
}

void CodeGenerator::prepareLocal(const SymbolTable &table) {
    FuncModel &f = funcModel();
    int pcnt = 0;
    for (const auto &ptre : table.getEntries()) {
        auto &e = *ptre;
        const auto &t = *e.getTypePtr();
        const auto &k = e.getKind();

        // allocate space
        int space = 0;
        switch (k) {
            case Kind::kConstantKind:
            case Kind::kVariableKind:
            case Kind::kParameterKind:
                if (t.isPrimitiveInteger()) {
                    space = 4;
                }  //
                else {
                    // TODO
                }
                break;
        }
        if (space > 0) f.forLocal(e, space);

        // assign value for constant
        dumpInstrs("// save param for %s\n", e.getNameCString());
        switch (k) {
            case Kind::kConstantKind:
                if (t.isPrimitiveInteger()) {
                    auto ptrcnst = e.getAttribute().constant();
                    dumpInstrs("    li t0, %d\n", ptrcnst->integer());
                    dumpInstrs("    sw t0, %d(s0)\n", e.stackLoc);
                }  //
                else {
                    // TODO
                }
                break;
            case Kind::kVariableKind:
                // nothing
                break;
            case Kind::kParameterKind:
                if (t.isPrimitiveInteger()) {
                    dumpInstrs("    sw %s, %d(s0)\n", regs[pcnt].c_str(), e.stackLoc);
                }  //
                else {
                    // TODO
                }
                pcnt++;
                break;
        }
        dumpInstrs("// saved\n");
    }
}

void CodeGenerator::beginFunc(const string &funcName, bool isGlobl = false) {
    currFunc = funcName;
    funcModels.emplace_back();

    dumpInstrs(".section    .text\n");
    dumpInstrs("    .align 2\n");
    if (isGlobl) dumpInstrs("    .globl %s\n", currFunc.c_str());
    dumpInstrs("    .type %s, @function\n", currFunc.c_str());
    dumpInstrs("%s:\n", currFunc.c_str());

    // move stack pointer to lower address to allocate a new stack
    dumpInstrs("    addi sp, sp, -128\n");
    // save return address of the caller function in the current stack
    dumpInstrs("    sw ra, 124(sp)\n");
    // save frame pointer of the last stack in the current stack
    dumpInstrs("    sw s0, 120(sp)\n");
    // move frame pointer to the bottom of the current stack
    dumpInstrs("    addi s0, sp, 128\n");
}

void CodeGenerator::endFunc() {
    // load return address saved in the current stack
    dumpInstrs("    lw ra, 124(sp)\n");
    // move frame pointer back to the bottom of the last stack
    dumpInstrs("    lw s0, 120(sp)\n");
    // move stack pointer back to the top of the last stack
    dumpInstrs("    addi sp, sp, 128\n");
    // jump back to the caller function
    dumpInstrs("    jr ra\n");

    dumpInstrs("    .size %s, .-%s\n", currFunc.c_str(), currFunc.c_str());

    funcModels.pop_back();
    currFunc = "";
}

void CodeGenerator::visit(ProgramNode &p_program) {
    // Reconstruct the hash table for looking up the symbol entry
    // Hint: Use symbol_manager->lookup(symbol_name) to get the symbol entry.
    symbol_manager->reconstructHashTableFromSymbolTable(p_program.getSymbolTable());

    // Generate RISC-V instructions for program header
    dumpHeader();

    // dump global variables
    dumpGlobl(*p_program.getSymbolTable());

    p_program.visitChildNodes(*this);

    // Remove the entries in the hash table
    symbol_manager->removeSymbolsFromHashTable(p_program.getSymbolTable());
}

void CodeGenerator::visit(DeclNode &p_decl) {
    // nothing to do
}

void CodeGenerator::visit(VariableNode &p_variable) {
    // nothing to do
}

void CodeGenerator::visit(ConstantValueNode &p_constant_value) {
    dumpInstrs("// const starts\n");
    auto ptr = p_constant_value.getTypePtr();
    auto t = *ptr;
    auto cnst = p_constant_value.getConstantPtr();
    if (t.isPrimitiveInteger()) {
        dumpInstrs("    li t0, %d\n", cnst->integer());
        pushToStackFrom("t0");
    } else {
        // TODO
    }
    dumpInstrs("// const ends\n");
}

void CodeGenerator::visit(FunctionNode &p_function) {
    // Reconstruct the hash table for looking up the symbol entry
    symbol_manager->reconstructHashTableFromSymbolTable(p_function.getSymbolTable());

    beginFunc(p_function.getName());
    prepareLocal(*p_function.getSymbolTable());

    p_function.visitChildNodes(*this);

    endFunc();

    // Remove the entries in the hash table
    symbol_manager->removeSymbolsFromHashTable(p_function.getSymbolTable());
}

void CodeGenerator::visit(CompoundStatementNode &p_compound_statement) {
    // Reconstruct the hash table for looking up the symbol entry
    symbol_manager->reconstructHashTableFromSymbolTable(p_compound_statement.getSymbolTable());
    bool beginMain = false;
    if (currFunc == "") {
        beginMain = true;
        beginFunc("main", true);
        prepareLocal(*p_compound_statement.getSymbolTable());
    }

    p_compound_statement.visitChildNodes(*this);

    if (beginMain) {
        endFunc();
    }

    // Remove the entries in the hash table
    symbol_manager->removeSymbolsFromHashTable(p_compound_statement.getSymbolTable());
}

void CodeGenerator::visit(PrintNode &p_print) {
    dumpInstrs("// print starts\n");
    p_print.visitChildNodes(*this);
    // now the value is in stack
    popFromStackTo("a0");
    saveRegs();
    dumpInstrs("    jal ra, printInt\n");
    loadRegs();
    dumpInstrs("// print ends\n");
}

void CodeGenerator::popFromStackTo(const char *reg) {
    dumpInstrs("    lw %s, 0(sp)\n", reg);
    dumpInstrs("    addi sp, sp, 4\n");
}

void CodeGenerator::pushToStackFrom(const char *reg) {
    dumpInstrs("    addi sp, sp, -4\n");
    dumpInstrs("    sw %s, 0(sp)\n", reg);
}

void CodeGenerator::visit(BinaryOperatorNode &p_bin_op) {
    dumpInstrs("// binary starts\n");
    p_bin_op.getLeftOperand()->accept(*this);
    p_bin_op.getRightOperand()->accept(*this);
    popFromStackTo("t1");
    popFromStackTo("t0");

    if (ifCond) {
        string b;
        switch (p_bin_op.getOp()) {
            case Operator::kEqualOp:
                b = "bne";
                break;
            case Operator::kLessOp:
                b = "bge";
                break;
            case Operator::kGreaterOp:
                b = "ble";
                break;
            case Operator::kLessOrEqualOp:
                b = "bgt";
                break;
            case Operator::kGreaterOrEqualOp:
                b = "blt";
                break;
        }
        dumpInstrs("    %s t0, t1, L%d\n", b.c_str(), elseLabel);
    }

    else {
        // assert is integer
        switch (p_bin_op.getOp()) {
            case Operator::kPlusOp:
                dumpInstrs("    add t0, t0, t1\n");
                break;
            case Operator::kMinusOp:
                dumpInstrs("    sub t0, t0, t1\n");
                break;
            case Operator::kModOp:
                dumpInstrs("    rem t0, t0, t1\n");
                break;
            case Operator::kDivideOp:
                dumpInstrs("    div t0, t0, t1\n");
                break;
            case Operator::kMultiplyOp:
                dumpInstrs("    mul t0, t0, t1\n");
                break;
        }
        pushToStackFrom("t0");
    }
    dumpInstrs("// binary ends\n");
}

void CodeGenerator::visit(UnaryOperatorNode &p_un_op) {
    p_un_op.visitChildNodes(*this);
    popFromStackTo("t0");
    // assert is integer
    switch (p_un_op.getOp()) {
        case Operator::kNegOp:
            dumpInstrs("    li  t1, -1\n");
            dumpInstrs("    mul t0, t0, t1\n");
            break;
    }
    pushToStackFrom("t0");
}

void CodeGenerator::visit(FunctionInvocationNode &p_func_invocation) {
    dumpInstrs("// invoke %s\n", p_func_invocation.getNameCString());
    // pass arguments
    auto &args = p_func_invocation.getArguments();
    for (int i = 0; i < args.size(); i++) {
        dumpInstrs("//// %dth arg\n", i);
        auto &arg = *args[i];
        arg.accept(*this);
        // popFromStackTo(regs[i].c_str());
    }
    for (int i = args.size() - 1; i >= 0; i--) {
        popFromStackTo(regs[i].c_str());
    }
    dumpInstrs("////\n");
    saveRegs();
    dumpInstrs("    jal ra, %s\n", p_func_invocation.getNameCString());
    loadRegs();
    pushToStackFrom("a0");
    dumpInstrs("// %s invoked\n", p_func_invocation.getNameCString());
}

void CodeGenerator::visit(VariableReferenceNode &p_variable_ref) {
    dumpInstrs("// var ref for %s starts\n", p_variable_ref.getNameCString());
    const char *varName = p_variable_ref.getNameCString();
    const SymbolEntry &e = *symbol_manager->lookup(varName);

    if (e.getTypePtr()->isPrimitiveInteger()) {
        // is global
        if (e.getLevel() == 0) {
            dumpInstrs("// var ref for %s is global\n", p_variable_ref.getNameCString());
            dumpInstrs("    la t0, %s\n", varName);
            dumpInstrs("    lw t0, 0(t0)\n");
        }  //
        else {
            // if is local
            dumpInstrs("// var ref for %s is local\n", p_variable_ref.getNameCString());
            dumpInstrs("    lw t0, %d(s0)\n", e.stackLoc);
        }
    } else {
        // TODO
    }
    pushToStackFrom("t0");
    dumpInstrs("// var ref ends\n");
}

void CodeGenerator::pushVarAddrToStack(const VariableReferenceNode &var) {
    // push addr
    const auto &e = symbol_manager->lookup(var.getNameCString());
    if (e->getLevel() == 0) {
        // global
        dumpInstrs("    la t0, %s\n", var.getNameCString());
    }  //
    else {
        // local
        dumpInstrs("    addi t0, s0, %d\n", e->stackLoc);
    }
    pushToStackFrom("t0");
}

void CodeGenerator::visit(AssignmentNode &p_assignment) {
    // save address of var
    dumpInstrs("// assignment starts\n");
    dumpInstrs("// get addr from var %s to stack\n", p_assignment.getLvalue()->getNameCString());
    pushVarAddrToStack(*p_assignment.getLvalue());

    // save expression value
    dumpInstrs("// get expression value to stack\n");
    auto e = p_assignment.getExpr();
    e->accept(*this);

    popFromStackTo("t1");
    popFromStackTo("t0");

    // assign t1 to t0
    dumpInstrs("// assign t1 to t0\n");
    dumpInstrs("    sw t1, 0(t0)\n");
    dumpInstrs("// assignment ends\n");
}

void CodeGenerator::visit(ReadNode &p_read) {
    pushVarAddrToStack(*p_read.getTarget());
    dumpInstrs("    jal ra, readInt\n");
    popFromStackTo("t0");
    dumpInstrs("    sw a0, 0(t0)\n");
}

void CodeGenerator::dumpLabel(int i) {
    dumpInstrs("L%d:\n", i);
}

void CodeGenerator::visit(IfNode &p_if) {
    auto cond = p_if.condition.get();
    auto body = p_if.body.get();
    auto elze = p_if.else_body.get();

    elseLabel = labelCnt++;
    int mainLabel = labelCnt++;
    doneLabel = labelCnt++;

    ifCond = true;
    cond->accept(*this);
    dumpLabel(mainLabel);
    body->accept(*this);
    dumpInstrs("    j L%d\n", doneLabel);
    dumpLabel(elseLabel);
    if (elze) elze->accept(*this);
    dumpLabel(doneLabel);
    ifCond = false;
}

void CodeGenerator::visit(WhileNode &p_while) {
}

void CodeGenerator::visit(ForNode &p_for) {
    // Reconstruct the hash table for looking up the symbol entry
    symbol_manager->reconstructHashTableFromSymbolTable(p_for.getSymbolTable());

    // Remove the entries in the hash table
    symbol_manager->removeSymbolsFromHashTable(p_for.getSymbolTable());
}

void CodeGenerator::visit(ReturnNode &p_return) {
    p_return.getRetval()->accept(*this);
    // assign value to a0
    popFromStackTo("a0");
}

void CodeGenerator::saveRegs(const char *a) {
    pushToStackFrom(a);
}

void CodeGenerator::saveRegs(const char *a, const char *b) {
    pushToStackFrom(a);
    pushToStackFrom(b);
}

void CodeGenerator::saveRegs(const char *a, const char *b, const char *c) {
    pushToStackFrom(a);
    pushToStackFrom(b);
    pushToStackFrom(c);
}

void CodeGenerator::loadRegs(const char *a) {
    popFromStackTo(a);
}

void CodeGenerator::loadRegs(const char *a, const char *b) {
    popFromStackTo(b);
    popFromStackTo(a);
}

void CodeGenerator::loadRegs(const char *a, const char *b, const char *c) {
    popFromStackTo(c);
    popFromStackTo(b);
    popFromStackTo(a);
}

void CodeGenerator::saveRegs() {
    // pushToStackFrom("sp");
    // pushToStackFrom("s0");
    /*  pushToStackFrom("t0");
    pushToStackFrom("t1");
    pushToStackFrom("a0");
    pushToStackFrom("a1");
    pushToStackFrom("a2");
    pushToStackFrom("a3");
    pushToStackFrom("a4");
    pushToStackFrom("a5");
    pushToStackFrom("a6");*/
    /*pushToStackFrom("t3");
    pushToStackFrom("t4");
    pushToStackFrom("t5");
    pushToStackFrom("t6");*/
}

void CodeGenerator::loadRegs() {
    /*popFromStackTo("t6");
    popFromStackTo("t5");
    popFromStackTo("t4");
    popFromStackTo("t3");*/
    /*popFromStackTo("a6");
    popFromStackTo("a5");
    popFromStackTo("a4");
    popFromStackTo("a3");
    popFromStackTo("a2");
    popFromStackTo("a1");
    popFromStackTo("a0");
    popFromStackTo("t1");
    popFromStackTo("t0");*/
    // popFromStackTo("s0");
    // popFromStackTo("sp");
}
