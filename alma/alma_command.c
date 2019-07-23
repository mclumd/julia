#include <stdlib.h>
#include "alma_command.h"
#include "tommyds/tommyds/tommytypes.h"
#include "tommyds/tommyds/tommyarray.h"
#include "tommyds/tommyds/tommyhashlin.h"
#include "tommyds/tommyds/tommylist.h"
#include "alma_kb.h"
#include "alma_formula.h"
#include "alma_parser.h"
#include "alma_print.h"

// Caller will need to free collection with kb_halt
void kb_init(kb **collection, char *file) {
  // Allocate and initialize
  *collection = malloc(sizeof(**collection));
  kb *collec = *collection;

  collec->time = 1;
  collec->prev_str = NULL;
  collec->now_str = NULL;
  collec->idling = 0;
  tommy_array_init(&collec->new_clauses);
  tommy_list_init(&collec->clauses);
  tommy_hashlin_init(&collec->index_map);
  tommy_hashlin_init(&collec->pos_map);
  tommy_list_init(&collec->pos_list);
  tommy_hashlin_init(&collec->neg_map);
  tommy_list_init(&collec->neg_list);
  tommy_hashlin_init(&collec->fif_map);
  tommy_array_init(&collec->res_tasks);
  tommy_hashlin_init(&collec->fif_tasks);
  tommy_list_init(&collec->backsearch_tasks);
  tommy_hashlin_init(&collec->distrusted);

  parse_init();
  assert_formula(collec, "now(1).", 0);

  // Given a file argument, obtain other initial clauses from
  if (file != NULL) {
    alma_node *trees;
    int num_trees;

    if (formulas_from_source(file, 1, &num_trees, &trees)) {
      nodes_to_clauses(trees, num_trees, &collec->new_clauses, 0);
      fif_to_front(&collec->new_clauses);
    }
    // If file cannot parse, cleanup and exit
    else {
      kb_halt(collec);
      exit(0);
    }
  }

  // Insert starting clauses
  for (tommy_size_t i = 0; i < tommy_array_size(&collec->new_clauses); i++) {
    clause *c = tommy_array_get(&collec->new_clauses, i);
    // Insert into KB if not duplicate
    if (duplicate_check(collec, c) == NULL)
      add_clause(collec, c);
    else
      free_clause(c);
  }
  tommy_array_done(&collec->new_clauses);
  tommy_array_init(&collec->new_clauses);

  // Generate starting resolution tasks
  tommy_node *i = tommy_list_head(&collec->clauses);
  while (i) {
    clause *c = ((index_mapping*)i->data)->value;
    if (c->tag == FIF)
      fif_task_map_init(collec, c);
    else {
      res_tasks_from_clause(collec, c, 0);
      fif_tasks_from_clause(collec, c);
    }
    i = i->next;
  }
}

// System idles if there are no resolution tasks, backsearch tasks, or to_unify fif values
// First of these is true when no new clauses (source of res tasks) exist
// To_unify values are all removed in current implementation each step from exhaustive fif
static int idling_check(kb *collection) {
  if (tommy_array_size(&collection->new_clauses) <= 1) {
    tommy_node *i = tommy_list_head(&collection->backsearch_tasks);
    while (i) {
      backsearch_task *bt = i->data;
      if (tommy_array_size(&bt->to_resolve) > 0)
        return 0;
      i = i->next;
    }
    return 1;
  }
  return 0;
}

void kb_step(kb *collection) {
  collection->time++;

  collection->now_str = now(collection->time);
  assert_formula(collection, collection->now_str, 0);
  if (collection->prev_str != NULL) {
    delete_formula(collection, collection->prev_str, 0);
    free(collection->prev_str);
  }
  else
    delete_formula(collection, "now(1).", 0);
  collection->prev_str = collection->now_str;

  process_res_tasks(collection, &collection->res_tasks, &collection->new_clauses, NULL);
  process_fif_tasks(collection);
  process_backsearch_tasks(collection);

  fif_to_front(&collection->new_clauses);
  // Insert new clauses derived that are not duplicates
  for (tommy_size_t i = 0; i < tommy_array_size(&collection->new_clauses); i++) {
    clause *c = tommy_array_get(&collection->new_clauses, i);
    clause *dupe = duplicate_check(collection, c);
    if (dupe == NULL) {
      res_tasks_from_clause(collection, c, 1);
      fif_tasks_from_clause(collection, c);

      // Get tasks between new KB clauses and all bs clauses
      tommy_node *curr = tommy_list_head(&collection->backsearch_tasks);
      while (curr) {
        backsearch_task *t = curr->data;
        for (int j = 0; j < tommy_array_size(&t->clauses); j++) {
          clause *bt_c = tommy_array_get(&t->clauses, j);
          for (int k = 0; k < bt_c->pos_count; k++)
            make_single_task(collection, bt_c, bt_c->pos_lits[k], c, &t->to_resolve, 1, 0);
          for (int k = 0; k < bt_c->neg_count; k++)
            make_single_task(collection, bt_c, bt_c->neg_lits[k], c, &t->to_resolve, 1, 1);
        }
        curr = curr->next;
      }

      add_clause(collection, c);

      if (c->parents != NULL) {
        int distrust = 0;
        // Update child info for parents of new clause, check for distrusted parents
        for (int j = 0; j < c->parents[0].count; j++) {
          add_child(c->parents[0].clauses[j], c);
          if (is_distrusted(collection, c->parents[0].clauses[j]->index))
            distrust = 1;
        }
        if (distrust) {
          char *time_str = long_to_str(collection->time);
          distrust_recursive(collection, c, time_str);
          free(time_str);
        }
      }
    }
    else {
      if (c->parents != NULL)
        transfer_parent(collection, dupe, c, 1);

      free_clause(c);
    }
  }

  // Generate new backsearch tasks
  tommy_node *i = tommy_list_head(&collection->backsearch_tasks);
  while (i) {
    generate_backsearch_tasks(collection, i->data);
    i = i->next;
  }

  if (idling_check(collection))
    collection->idling = 1;

  if (collection->idling)
    printf("Idling...\n");

  tommy_array_done(&collection->new_clauses);
  tommy_array_init(&collection->new_clauses);
}

void kb_print(kb *collection) {
  tommy_node *i = tommy_list_head(&collection->clauses);
  while (i) {
    index_mapping *data = i->data;
    printf("%ld: ", data->key);
    clause_print(data->value);
    printf("\n");
    i = i->next;
  }

  i = tommy_list_head(&collection->backsearch_tasks);
  if (i) {
    printf("Back searches:\n");
    for (int count = 0; i != NULL; i = i->next, count++) {
      printf("%d\n", count);
      backsearch_task *t = i->data;
      for (tommy_size_t j = 0; j < tommy_array_size(&t->clauses); j++) {
        clause *c = tommy_array_get(&t->clauses, j);
        printf("%ld: ", c->index);
        clause_print(c);
        binding_mapping *m = tommy_hashlin_search(&t->clause_bindings, bm_compare, &c->index, tommy_hash_u64(0, &c->index, sizeof(c->index)));
        if (m != NULL) {
          printf(" (bindings: ");
          print_bindings(m->bindings);
          printf(")");
        }
        printf("\n");
      }
    }
  }
  printf("\n");
}

void kb_halt(kb *collection) {
  // now_str and prev_str alias at this point, only free one
  free(collection->now_str);

  for (tommy_size_t i = 0; i < tommy_array_size(&collection->new_clauses); i++)
    free_clause(tommy_array_get(&collection->new_clauses, i));
  tommy_array_done(&collection->new_clauses);

  tommy_node *curr = tommy_list_head(&collection->clauses);
  while (curr) {
    index_mapping *data = curr->data;
    curr = curr->next;
    free_clause(data->value);
    free(data);
  }
  tommy_hashlin_done(&collection->index_map);

  tommy_list_foreach(&collection->pos_list, free_predname_mapping);
  tommy_hashlin_done(&collection->pos_map);

  tommy_list_foreach(&collection->neg_list, free_predname_mapping);
  tommy_hashlin_done(&collection->neg_map);

  tommy_hashlin_foreach(&collection->fif_map, free_fif_mapping);
  tommy_hashlin_done(&collection->fif_map);

  // Res task pointers are aliases to those freed from collection->clauses, so only free overall task here
  for (tommy_size_t i = 0; i < tommy_array_size(&collection->res_tasks); i++)
    free(tommy_array_get(&collection->res_tasks, i));
  tommy_array_done(&collection->res_tasks);

  tommy_hashlin_foreach(&collection->fif_tasks, free_fif_task_mapping);
  tommy_hashlin_done(&collection->fif_tasks);

  curr = tommy_list_head(&collection->backsearch_tasks);
  while (curr) {
    backsearch_task *data = curr->data;
    curr = curr->next;
    backsearch_halt(data);
  }

  tommy_hashlin_foreach(&collection->distrusted, free);
  tommy_hashlin_done(&collection->distrusted);

  free(collection);

  parse_cleanup();
}

void kb_assert(kb *collection, char *string) {
  assert_formula(collection, string, 1);
}

void kb_remove(kb *collection, char *string) {
  delete_formula(collection, string, 1);
}

void kb_backsearch(kb *collection, char *string) {
  // Parse string into clauses
  alma_node *formulas;
  int formula_count;
  if (formulas_from_source(string, 0, &formula_count, &formulas)) {
    tommy_array arr;
    tommy_array_init(&arr);
    flatten_node(formulas, &arr, 0);
    clause *c = tommy_array_get(&arr, 0);

    for (int i = 0; i < formula_count; i++)
      free_alma_tree(formulas+i);
    free(formulas);
    for (int i = 1; i < tommy_array_size(&arr); i++)
      free_clause(tommy_array_get(&arr, i));
    tommy_array_done(&arr);

    if (c->pos_count + c->neg_count > 1) {
      printf("query clause has too many literals\n");
      free_clause(c);
    }
    else {
      collection->idling = 0;
      backsearch_from_clause(collection, c);
    }
  }
}
