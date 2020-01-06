#ifndef alma_formula_h
#define alma_formula_h

#include "mpc/mpc.h"

extern long long variable_id_count;

typedef enum node_type {FOL, PREDICATE} node_type;

struct alma_fol;
struct alma_pred;

typedef struct alma_node {
  node_type type;
  union {
    struct alma_fol *fol;
    struct alma_function *predicate;
  };
} alma_node;

typedef enum alma_operator {NOT, OR, AND, IF} alma_operator;
typedef enum if_tag {NONE, FIF, BIF} if_tag;

typedef struct alma_fol {
  alma_operator op; // Which arguments are used is implicit based on operator
  alma_node *arg1; // Holds antecedent when op is IF
  alma_node *arg2; // Holds consequent when op is IF
  if_tag tag; // Used to record FIF/BIF tag for later use
} alma_fol;

typedef struct alma_function {
  char *name;
  int term_count;
  struct alma_term *terms;
} alma_function;

typedef enum term_type {VARIABLE, FUNCTION} term_type;

struct alma_variable;
struct alma_function;

typedef struct alma_term {
  term_type type;
  union {
    struct alma_variable *variable;
    alma_function *function;
  };
} alma_term;

typedef struct alma_variable {
  char *name;
  long long id; // Not initialized until the variable in which it appears is converted into a clause
} alma_variable;

// TODO: Determine which of this file's functions should have static linkage

void alma_fol_init(alma_node *node, alma_operator op, alma_node *arg1, alma_node *arg2, if_tag tag);
void alma_term_init(alma_term *term, mpc_ast_t *ast);
void alma_function_init(alma_function *func, mpc_ast_t *ast);
void alma_predicate_init(alma_node *node, mpc_ast_t *ast);

int formulas_from_source(char *source, int file_src, int *formula_count, alma_node **formulas);
void generate_alma_trees(mpc_ast_t *ast, alma_node **alma_trees, int *size);
void free_function(alma_function *func);
void free_term(alma_term *term);
void free_alma_tree(alma_node *node);

void copy_alma_var(alma_variable *original, alma_variable *copy);
void copy_alma_term(alma_term *original, alma_term *copy);
void copy_alma_function(alma_function *original, alma_function *copy);
void copy_alma_tree(alma_node *original, alma_node *copy);

void eliminate_conditionals(alma_node *node);
void negation_inwards(alma_node *node);
void dist_or_over_and(alma_node *node);
void make_cnf(alma_node *node);

#endif
