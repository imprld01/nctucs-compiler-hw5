#ifndef __CODE_GENERATOR_H
#define __CODE_GENERATOR_H

#include <map>
#include <set>
#include <utility>
#include <vector>

#include "sema/SymbolTable.hpp"
#include "visitor/AstNodeVisitor.hpp"

using std::map;
using std::pair;
using std::set;
using std::string;

class FuncModel {
   private:
    int stackpt = -8;

   public:
    void forLocal(SymbolEntry &, int);
};

class CodeGenerator : public AstNodeVisitor {
   public:
    CodeGenerator(const char *in_file_name, const char *out_file_name, SymbolManager *symbol_manager);
    ~CodeGenerator();

    void visit(ProgramNode &p_program) override;
    void visit(DeclNode &p_decl) override;
    void visit(VariableNode &p_variable) override;
    void visit(ConstantValueNode &p_constant_value) override;
    void visit(FunctionNode &p_function) override;
    void visit(CompoundStatementNode &p_compound_statement) override;
    void visit(PrintNode &p_print) override;
    void visit(BinaryOperatorNode &p_bin_op) override;
    void visit(UnaryOperatorNode &p_un_op) override;
    void visit(FunctionInvocationNode &p_func_invocation) override;
    void visit(VariableReferenceNode &p_variable_ref) override;
    void visit(AssignmentNode &p_assignment) override;
    void visit(ReadNode &p_read) override;
    void visit(IfNode &p_if) override;
    void visit(WhileNode &p_while) override;
    void visit(ForNode &p_for) override;
    void visit(ReturnNode &p_return) override;

   private:
    const char *in_file_name;
    FILE *out_fp;
    SymbolManager *symbol_manager;
    void dumpInstrs(const char *format, ...);

    void dumpHeader();
    void dumpGloblVar(const string &, const PType &);
    void dumpGloblConst(const string &, const Constant &);
    void dumpGlobl(const SymbolTable &);

    void beginFunc(const string &, bool);
    string currFunc = "";
    void endFunc();

    void prepareLocal(const SymbolTable &);
    std::vector<FuncModel> funcModels;
    FuncModel &funcModel();

    void popFromStackTo(const char *);
    void pushToStackFrom(const char *);
    void pushVarAddrToStack(const VariableReferenceNode &);

    void varAddr(const VariableReferenceNode &);
    void saveRegs(const char *);
    void saveRegs(const char *, const char *);
    void saveRegs(const char *, const char *, const char *);
    void loadRegs(const char *);
    void loadRegs(const char *, const char *);
    void loadRegs(const char *, const char *, const char *);

    const std::vector<string> regs = {
        "a0",
        "a1",
        "a2",
        "a3",
        "a4",
        "a5",
        "a6",
        "a7",
        "a8",
        "t3",
        "t4",
        "t5",
        "t6",
    };
};

#endif