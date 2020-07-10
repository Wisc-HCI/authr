/* A Bison parser, made by GNU Bison 3.0.4.  */

/* Bison implementation for Yacc-like parsers in C

   Copyright (C) 1984, 1989-1990, 2000-2015 Free Software Foundation, Inc.

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.  */

/* As a special exception, you may create a larger work that contains
   part or all of the Bison parser skeleton and distribute that work
   under terms of your choice, so long as that work isn't itself a
   parser generator using the skeleton or a modified version thereof
   as a parser skeleton.  Alternatively, if you modify or redistribute
   the parser skeleton itself, you may (at your option) remove this
   special exception, which will cause the skeleton and the resulting
   Bison output files to be licensed under the GNU General Public
   License without this special exception.

   This special exception was added by the Free Software Foundation in
   version 2.2 of Bison.  */

/* C LALR(1) parser skeleton written by Richard Stallman, by
   simplifying the original so-called "semantic" parser.  */

/* All symbols defined below should begin with yy or YY, to avoid
   infringing on user name space.  This should be done even for local
   variables, as they might otherwise be expanded by user macros.
   There are some unavoidable exceptions within include files to
   define necessary library symbols; they are noted "INFRINGES ON
   USER NAME SPACE" below.  */

/* Identify Bison output.  */
#define YYBISON 1

/* Bison version.  */
#define YYBISON_VERSION "3.0.4"

/* Skeleton name.  */
#define YYSKELETON_NAME "yacc.c"

/* Pure parsers.  */
#define YYPURE 0

/* Push parsers.  */
#define YYPUSH 0

/* Pull parsers.  */
#define YYPULL 1


/* Substitute the variable and function names.  */
#define yyparse         pddl_parse
#define yylex           pddl_lex
#define yyerror         pddl_error
#define yydebug         pddl_debug
#define yynerrs         pddl_nerrs

#define yylval          pddl_lval
#define yychar          pddl_char

/* Copy the first part of user declarations.  */
#line 9 "parser.y" /* yacc.c:339  */

#include "cpt.h"
#include "options.h"
#include "trace.h"
#include "pddl.h"
#include "structs.h"
#include "plan.h"
#include "globs.h"

extern int pddl_lex(PDDLDomain *domain);
extern char *pddl_text;
extern FILE *pddl_in;

#define pddl_error(domain, s) error(parser, "Parser error (%s) at line %d : %s", s, parser_get_lineno(), pddl_text)

#line 90 "/home/john/Desktop/PDDL/planner/descarwin/build/cpt-yahsp/src/parser.tab.cpp" /* yacc.c:339  */

# ifndef YY_NULLPTR
#  if defined __cplusplus && 201103L <= __cplusplus
#   define YY_NULLPTR nullptr
#  else
#   define YY_NULLPTR 0
#  endif
# endif

/* Enabling verbose error messages.  */
#ifdef YYERROR_VERBOSE
# undef YYERROR_VERBOSE
# define YYERROR_VERBOSE 1
#else
# define YYERROR_VERBOSE 0
#endif

/* In a future release of Bison, this section will be replaced
   by #include "parser.tab.hpp".  */
#ifndef YY_PDDL_HOME_JOHN_DESKTOP_PDDL_PLANNER_DESCARWIN_BUILD_CPT_YAHSP_SRC_PARSER_TAB_HPP_INCLUDED
# define YY_PDDL_HOME_JOHN_DESKTOP_PDDL_PLANNER_DESCARWIN_BUILD_CPT_YAHSP_SRC_PARSER_TAB_HPP_INCLUDED
/* Debug traces.  */
#ifndef YYDEBUG
# define YYDEBUG 0
#endif
#if YYDEBUG
extern int pddl_debug;
#endif

/* Token type.  */
#ifndef YYTOKENTYPE
# define YYTOKENTYPE
  enum yytokentype
  {
    DEFINE_TOK = 258,
    DOMAIN_TOK = 259,
    REQUIREMENTS_TOK = 260,
    TYPES_TOK = 261,
    CONSTANTS_TOK = 262,
    PREDICATES_TOK = 263,
    FUNCTIONS_TOK = 264,
    PROBLEM_TOK = 265,
    DOMNAME_TOK = 266,
    OBJECTS_TOK = 267,
    INIT_TOK = 268,
    GOAL_TOK = 269,
    METRIC_TOK = 270,
    ACTION_TOK = 271,
    PARAMETERS_TOK = 272,
    VARS_TOK = 273,
    DURATION_TOK = 274,
    REAL_DURATION_TOK = 275,
    PRECONDITION_TOK = 276,
    EFFECT_TOK = 277,
    CONSTRAINTS_TOK = 278,
    EITHER_TOK = 279,
    AND_TOK = 280,
    NOT_TOK = 281,
    ATSTART_TOK = 282,
    ATEND_TOK = 283,
    OVERALL_TOK = 284,
    EQ_TOK = 285,
    LTE_TOK = 286,
    GTE_TOK = 287,
    LT_TOK = 288,
    GT_TOK = 289,
    INC_TOK = 290,
    DEC_TOK = 291,
    ASS_TOK = 292,
    ADD_TOK = 293,
    SUB_TOK = 294,
    MUL_TOK = 295,
    DIV_TOK = 296,
    FAM_TOK = 297,
    LP = 298,
    RP = 299,
    NAME_TOK = 300,
    NUMBER_TOK = 301
  };
#endif

/* Value type.  */
#if ! defined YYSTYPE && ! defined YYSTYPE_IS_DECLARED

union YYSTYPE
{
#line 46 "parser.y" /* yacc.c:355  */

  char *string;
  int number;
  gdsl_list_t list;
  PDDLOperator *ope;

#line 184 "/home/john/Desktop/PDDL/planner/descarwin/build/cpt-yahsp/src/parser.tab.cpp" /* yacc.c:355  */
};

typedef union YYSTYPE YYSTYPE;
# define YYSTYPE_IS_TRIVIAL 1
# define YYSTYPE_IS_DECLARED 1
#endif


extern YYSTYPE pddl_lval;

int pddl_parse (PDDLDomain *domain);

#endif /* !YY_PDDL_HOME_JOHN_DESKTOP_PDDL_PLANNER_DESCARWIN_BUILD_CPT_YAHSP_SRC_PARSER_TAB_HPP_INCLUDED  */

/* Copy the second part of user declarations.  */

#line 201 "/home/john/Desktop/PDDL/planner/descarwin/build/cpt-yahsp/src/parser.tab.cpp" /* yacc.c:358  */

#ifdef short
# undef short
#endif

#ifdef YYTYPE_UINT8
typedef YYTYPE_UINT8 yytype_uint8;
#else
typedef unsigned char yytype_uint8;
#endif

#ifdef YYTYPE_INT8
typedef YYTYPE_INT8 yytype_int8;
#else
typedef signed char yytype_int8;
#endif

#ifdef YYTYPE_UINT16
typedef YYTYPE_UINT16 yytype_uint16;
#else
typedef unsigned short int yytype_uint16;
#endif

#ifdef YYTYPE_INT16
typedef YYTYPE_INT16 yytype_int16;
#else
typedef short int yytype_int16;
#endif

#ifndef YYSIZE_T
# ifdef __SIZE_TYPE__
#  define YYSIZE_T __SIZE_TYPE__
# elif defined size_t
#  define YYSIZE_T size_t
# elif ! defined YYSIZE_T
#  include <stddef.h> /* INFRINGES ON USER NAME SPACE */
#  define YYSIZE_T size_t
# else
#  define YYSIZE_T unsigned int
# endif
#endif

#define YYSIZE_MAXIMUM ((YYSIZE_T) -1)

#ifndef YY_
# if defined YYENABLE_NLS && YYENABLE_NLS
#  if ENABLE_NLS
#   include <libintl.h> /* INFRINGES ON USER NAME SPACE */
#   define YY_(Msgid) dgettext ("bison-runtime", Msgid)
#  endif
# endif
# ifndef YY_
#  define YY_(Msgid) Msgid
# endif
#endif

#ifndef YY_ATTRIBUTE
# if (defined __GNUC__                                               \
      && (2 < __GNUC__ || (__GNUC__ == 2 && 96 <= __GNUC_MINOR__)))  \
     || defined __SUNPRO_C && 0x5110 <= __SUNPRO_C
#  define YY_ATTRIBUTE(Spec) __attribute__(Spec)
# else
#  define YY_ATTRIBUTE(Spec) /* empty */
# endif
#endif

#ifndef YY_ATTRIBUTE_PURE
# define YY_ATTRIBUTE_PURE   YY_ATTRIBUTE ((__pure__))
#endif

#ifndef YY_ATTRIBUTE_UNUSED
# define YY_ATTRIBUTE_UNUSED YY_ATTRIBUTE ((__unused__))
#endif

#if !defined _Noreturn \
     && (!defined __STDC_VERSION__ || __STDC_VERSION__ < 201112)
# if defined _MSC_VER && 1200 <= _MSC_VER
#  define _Noreturn __declspec (noreturn)
# else
#  define _Noreturn YY_ATTRIBUTE ((__noreturn__))
# endif
#endif

/* Suppress unused-variable warnings by "using" E.  */
#if ! defined lint || defined __GNUC__
# define YYUSE(E) ((void) (E))
#else
# define YYUSE(E) /* empty */
#endif

#if defined __GNUC__ && 407 <= __GNUC__ * 100 + __GNUC_MINOR__
/* Suppress an incorrect diagnostic about yylval being uninitialized.  */
# define YY_IGNORE_MAYBE_UNINITIALIZED_BEGIN \
    _Pragma ("GCC diagnostic push") \
    _Pragma ("GCC diagnostic ignored \"-Wuninitialized\"")\
    _Pragma ("GCC diagnostic ignored \"-Wmaybe-uninitialized\"")
# define YY_IGNORE_MAYBE_UNINITIALIZED_END \
    _Pragma ("GCC diagnostic pop")
#else
# define YY_INITIAL_VALUE(Value) Value
#endif
#ifndef YY_IGNORE_MAYBE_UNINITIALIZED_BEGIN
# define YY_IGNORE_MAYBE_UNINITIALIZED_BEGIN
# define YY_IGNORE_MAYBE_UNINITIALIZED_END
#endif
#ifndef YY_INITIAL_VALUE
# define YY_INITIAL_VALUE(Value) /* Nothing. */
#endif


#if ! defined yyoverflow || YYERROR_VERBOSE

/* The parser invokes alloca or malloc; define the necessary symbols.  */

# ifdef YYSTACK_USE_ALLOCA
#  if YYSTACK_USE_ALLOCA
#   ifdef __GNUC__
#    define YYSTACK_ALLOC __builtin_alloca
#   elif defined __BUILTIN_VA_ARG_INCR
#    include <alloca.h> /* INFRINGES ON USER NAME SPACE */
#   elif defined _AIX
#    define YYSTACK_ALLOC __alloca
#   elif defined _MSC_VER
#    include <malloc.h> /* INFRINGES ON USER NAME SPACE */
#    define alloca _alloca
#   else
#    define YYSTACK_ALLOC alloca
#    if ! defined _ALLOCA_H && ! defined EXIT_SUCCESS
#     include <stdlib.h> /* INFRINGES ON USER NAME SPACE */
      /* Use EXIT_SUCCESS as a witness for stdlib.h.  */
#     ifndef EXIT_SUCCESS
#      define EXIT_SUCCESS 0
#     endif
#    endif
#   endif
#  endif
# endif

# ifdef YYSTACK_ALLOC
   /* Pacify GCC's 'empty if-body' warning.  */
#  define YYSTACK_FREE(Ptr) do { /* empty */; } while (0)
#  ifndef YYSTACK_ALLOC_MAXIMUM
    /* The OS might guarantee only one guard page at the bottom of the stack,
       and a page size can be as small as 4096 bytes.  So we cannot safely
       invoke alloca (N) if N exceeds 4096.  Use a slightly smaller number
       to allow for a few compiler-allocated temporary stack slots.  */
#   define YYSTACK_ALLOC_MAXIMUM 4032 /* reasonable circa 2006 */
#  endif
# else
#  define YYSTACK_ALLOC YYMALLOC
#  define YYSTACK_FREE YYFREE
#  ifndef YYSTACK_ALLOC_MAXIMUM
#   define YYSTACK_ALLOC_MAXIMUM YYSIZE_MAXIMUM
#  endif
#  if (defined __cplusplus && ! defined EXIT_SUCCESS \
       && ! ((defined YYMALLOC || defined malloc) \
             && (defined YYFREE || defined free)))
#   include <stdlib.h> /* INFRINGES ON USER NAME SPACE */
#   ifndef EXIT_SUCCESS
#    define EXIT_SUCCESS 0
#   endif
#  endif
#  ifndef YYMALLOC
#   define YYMALLOC malloc
#   if ! defined malloc && ! defined EXIT_SUCCESS
void *malloc (YYSIZE_T); /* INFRINGES ON USER NAME SPACE */
#   endif
#  endif
#  ifndef YYFREE
#   define YYFREE free
#   if ! defined free && ! defined EXIT_SUCCESS
void free (void *); /* INFRINGES ON USER NAME SPACE */
#   endif
#  endif
# endif
#endif /* ! defined yyoverflow || YYERROR_VERBOSE */


#if (! defined yyoverflow \
     && (! defined __cplusplus \
         || (defined YYSTYPE_IS_TRIVIAL && YYSTYPE_IS_TRIVIAL)))

/* A type that is properly aligned for any stack member.  */
union yyalloc
{
  yytype_int16 yyss_alloc;
  YYSTYPE yyvs_alloc;
};

/* The size of the maximum gap between one aligned stack and the next.  */
# define YYSTACK_GAP_MAXIMUM (sizeof (union yyalloc) - 1)

/* The size of an array large to enough to hold all stacks, each with
   N elements.  */
# define YYSTACK_BYTES(N) \
     ((N) * (sizeof (yytype_int16) + sizeof (YYSTYPE)) \
      + YYSTACK_GAP_MAXIMUM)

# define YYCOPY_NEEDED 1

/* Relocate STACK from its old location to the new one.  The
   local variables YYSIZE and YYSTACKSIZE give the old and new number of
   elements in the stack, and YYPTR gives the new location of the
   stack.  Advance YYPTR to a properly aligned location for the next
   stack.  */
# define YYSTACK_RELOCATE(Stack_alloc, Stack)                           \
    do                                                                  \
      {                                                                 \
        YYSIZE_T yynewbytes;                                            \
        YYCOPY (&yyptr->Stack_alloc, Stack, yysize);                    \
        Stack = &yyptr->Stack_alloc;                                    \
        yynewbytes = yystacksize * sizeof (*Stack) + YYSTACK_GAP_MAXIMUM; \
        yyptr += yynewbytes / sizeof (*yyptr);                          \
      }                                                                 \
    while (0)

#endif

#if defined YYCOPY_NEEDED && YYCOPY_NEEDED
/* Copy COUNT objects from SRC to DST.  The source and destination do
   not overlap.  */
# ifndef YYCOPY
#  if defined __GNUC__ && 1 < __GNUC__
#   define YYCOPY(Dst, Src, Count) \
      __builtin_memcpy (Dst, Src, (Count) * sizeof (*(Src)))
#  else
#   define YYCOPY(Dst, Src, Count)              \
      do                                        \
        {                                       \
          YYSIZE_T yyi;                         \
          for (yyi = 0; yyi < (Count); yyi++)   \
            (Dst)[yyi] = (Src)[yyi];            \
        }                                       \
      while (0)
#  endif
# endif
#endif /* !YYCOPY_NEEDED */

/* YYFINAL -- State number of the termination state.  */
#define YYFINAL  4
/* YYLAST -- Last index in YYTABLE.  */
#define YYLAST   223

/* YYNTOKENS -- Number of terminals.  */
#define YYNTOKENS  47
/* YYNNTS -- Number of nonterminals.  */
#define YYNNTS  21
/* YYNRULES -- Number of rules.  */
#define YYNRULES  75
/* YYNSTATES -- Number of states.  */
#define YYNSTATES  175

/* YYTRANSLATE[YYX] -- Symbol number corresponding to YYX as returned
   by yylex, with out-of-bounds checking.  */
#define YYUNDEFTOK  2
#define YYMAXUTOK   301

#define YYTRANSLATE(YYX)                                                \
  ((unsigned int) (YYX) <= YYMAXUTOK ? yytranslate[YYX] : YYUNDEFTOK)

/* YYTRANSLATE[TOKEN-NUM] -- Symbol number corresponding to TOKEN-NUM
   as returned by yylex, without out-of-bounds checking.  */
static const yytype_uint8 yytranslate[] =
{
       0,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     1,     2,     3,     4,
       5,     6,     7,     8,     9,    10,    11,    12,    13,    14,
      15,    16,    17,    18,    19,    20,    21,    22,    23,    24,
      25,    26,    27,    28,    29,    30,    31,    32,    33,    34,
      35,    36,    37,    38,    39,    40,    41,    42,    43,    44,
      45,    46
};

#if YYDEBUG
  /* YYRLINE[YYN] -- Source line where rule number YYN was defined.  */
static const yytype_uint8 yyrline[] =
{
       0,    56,    56,    57,    60,    61,    62,    63,    64,    65,
      66,    69,    70,    71,    72,    73,    74,    75,    77,    82,
      86,    87,    88,    89,    90,    91,    92,    93,    97,    98,
     102,   103,   104,   105,   109,   110,   114,   115,   116,   120,
     121,   122,   123,   127,   128,   129,   133,   134,   145,   146,
     150,   151,   152,   153,   157,   158,   162,   163,   167,   168,
     169,   170,   171,   172,   173,   174,   178,   179,   180,   184,
     185,   186,   187,   188,   192,   193
};
#endif

#if YYDEBUG || YYERROR_VERBOSE || 0
/* YYTNAME[SYMBOL-NUM] -- String name of the symbol SYMBOL-NUM.
   First, the terminals, then, starting at YYNTOKENS, nonterminals.  */
static const char *const yytname[] =
{
  "$end", "error", "$undefined", "DEFINE_TOK", "DOMAIN_TOK",
  "REQUIREMENTS_TOK", "TYPES_TOK", "CONSTANTS_TOK", "PREDICATES_TOK",
  "FUNCTIONS_TOK", "PROBLEM_TOK", "DOMNAME_TOK", "OBJECTS_TOK", "INIT_TOK",
  "GOAL_TOK", "METRIC_TOK", "ACTION_TOK", "PARAMETERS_TOK", "VARS_TOK",
  "DURATION_TOK", "REAL_DURATION_TOK", "PRECONDITION_TOK", "EFFECT_TOK",
  "CONSTRAINTS_TOK", "EITHER_TOK", "AND_TOK", "NOT_TOK", "ATSTART_TOK",
  "ATEND_TOK", "OVERALL_TOK", "EQ_TOK", "LTE_TOK", "GTE_TOK", "LT_TOK",
  "GT_TOK", "INC_TOK", "DEC_TOK", "ASS_TOK", "ADD_TOK", "SUB_TOK",
  "MUL_TOK", "DIV_TOK", "FAM_TOK", "LP", "RP", "NAME_TOK", "NUMBER_TOK",
  "$accept", "domain", "domain_definition", "problem_definition", "action",
  "action_body", "constraint_list", "formula", "atom_list",
  "typed_atom_list", "atom", "basic_atom", "simplest_atom",
  "typed_token_list", "typed_token_list_aux", "type", "token_list",
  "function", "expression", "operation", "expression_list", YY_NULLPTR
};
#endif

# ifdef YYPRINT
/* YYTOKNUM[NUM] -- (External) token number corresponding to the
   (internal) symbol number NUM (which must be that of a token).  */
static const yytype_uint16 yytoknum[] =
{
       0,   256,   257,   258,   259,   260,   261,   262,   263,   264,
     265,   266,   267,   268,   269,   270,   271,   272,   273,   274,
     275,   276,   277,   278,   279,   280,   281,   282,   283,   284,
     285,   286,   287,   288,   289,   290,   291,   292,   293,   294,
     295,   296,   297,   298,   299,   300,   301
};
# endif

#define YYPACT_NINF -54

#define yypact_value_is_default(Yystate) \
  (!!((Yystate) == (-54)))

#define YYTABLE_NINF -59

#define yytable_value_is_error(Yytable_value) \
  0

  /* YYPACT[STATE-NUM] -- Index in YYTABLE of the portion describing
     STATE-NUM.  */
static const yytype_int16 yypact[] =
{
     -39,     9,    16,     6,   -54,     1,     8,    29,    22,    45,
     -54,   -54,   108,   123,   137,   -54,   -54,    73,   -54,   -54,
       7,     7,   -54,   -54,    55,   102,     7,   -54,    75,    -5,
     112,    63,    78,    23,   116,   -54,   120,   125,   127,   -54,
     136,   141,   129,   104,   142,   -54,   -54,   -54,   162,    21,
     163,   -54,   -54,   165,   -54,   145,    78,   -54,   -54,   -54,
      -1,   -54,   -54,   -54,   152,    -2,   -54,   -54,   -54,   -54,
     149,   150,   150,   150,     2,   -54,   -54,   -54,   -54,   -54,
     -54,   -54,   -54,     7,   149,   -54,   151,    18,   -54,   -54,
     153,   -54,   -54,   -54,    53,   154,   155,    47,    21,    75,
      75,   157,   -54,   131,    -6,   158,    46,   159,   160,   161,
     164,   166,    21,   156,     7,   -54,   -54,   -54,   -54,   -54,
      21,   -54,   133,   134,    78,   -54,   -54,     7,    64,   -54,
     -54,   -54,   -54,   170,   -54,   -54,   -54,   -54,   -54,   -54,
     -54,   167,    21,   -54,   115,   168,   171,   -54,   145,   172,
      80,   -54,   -54,   173,   -54,   -54,   169,   -54,   -54,   -54,
     111,   138,   175,    21,   176,   -54,   174,   119,   -54,    21,
     -54,   177,   178,   179,   -54
};

  /* YYDEFACT[STATE-NUM] -- Default reduction number in state STATE-NUM.
     Performed when YYTABLE does not specify something else to do.  Zero
     means the default is an error.  */
static const yytype_uint8 yydefact[] =
{
       0,     0,     0,     0,     1,     0,     0,     0,     0,     0,
       4,    11,     0,     0,     0,     2,    10,     0,     3,    56,
      50,    50,    34,    36,     0,     0,    50,    34,    30,     0,
       0,     0,     0,    50,     0,    49,     0,     0,     0,    20,
       0,     0,     0,     0,     0,    32,    39,    43,     0,     0,
       0,     5,    57,     0,    54,    50,     0,    51,     6,     7,
       0,     8,    35,     9,    37,     0,    12,    13,    14,    34,
       0,     0,     0,     0,    50,    59,    60,    61,    62,    63,
      64,    65,    31,    50,     0,    15,     0,     0,    66,    67,
       0,    28,    56,    48,    50,     0,     0,     0,     0,    30,
      30,     0,    19,     0,     0,     0,     0,     0,     0,     0,
       0,     0,     0,     0,    50,    69,    70,    71,    72,    73,
       0,    17,     0,     0,     0,    52,    38,    50,     0,    23,
      24,    25,    26,     0,    33,    44,    40,    41,    42,    47,
      46,     0,     0,    74,     0,     0,     0,    55,    50,     0,
      50,    28,    45,     0,    68,    75,     0,    16,    53,    21,
      50,     0,     0,     0,     0,    27,     0,     0,    22,     0,
      29,     0,     0,     0,    18
};

  /* YYPGOTO[NTERM-NUM].  */
static const yytype_int8 yypgoto[] =
{
     -54,   -54,   -54,   -54,   -54,   -54,    56,    84,   -26,   -54,
     -28,    39,   -47,   -13,   -33,   -53,   114,   -54,   -43,   -54,
      33
};

  /* YYDEFGOTO[NTERM-NUM].  */
static const yytype_int16 yydefgoto[] =
{
      -1,     2,    12,    13,    16,    65,   122,    44,    37,    38,
      45,    46,    47,   110,    35,    55,    31,    84,   143,   120,
     144
};

  /* YYTABLE[YYPACT[STATE-NUM]] -- What to do in state STATE-NUM.  If
     positive, shift that token.  If negative, reduce the rule whose
     number is the opposite.  If YYTABLE_NINF, syntax error.  */
static const yytype_int16 yytable[] =
{
      57,    42,    89,    94,     1,     6,    90,    34,    36,    62,
      64,     7,     3,    41,    62,    96,     4,    97,    98,    99,
     100,   101,    93,   105,   114,    70,    71,    72,    73,    74,
      75,    76,    77,    78,    79,    80,    81,   112,    48,    83,
      49,    32,   102,   103,    83,   -58,    32,    33,   114,     5,
      89,    89,    33,     8,   129,   130,   115,   116,   117,   118,
     119,   125,    56,    83,    87,    89,    10,    88,    33,   141,
     111,   148,    70,    89,     9,    62,    74,    75,    76,    77,
      78,    79,    80,    81,    25,    26,    27,    28,    29,    11,
     128,    83,   124,    88,   150,    89,    30,    89,    33,   153,
      39,   155,   115,   116,   117,   118,   119,    51,    52,    83,
     107,   108,   109,    89,   149,   158,    89,   164,    43,    32,
      89,    53,    89,    54,   155,   160,   171,    57,   163,    69,
      70,    71,    72,    73,    74,    75,    76,    77,    78,    79,
      80,    81,    19,    20,    21,    22,    23,    40,    82,    83,
      56,    14,    15,    24,    87,    50,    33,    88,    87,   154,
      58,    88,    87,   170,    59,    88,    17,    18,    60,    61,
      60,    63,    60,    68,    60,   134,   145,   146,   147,    52,
      66,   145,   165,   131,   132,    67,    85,    86,    91,    92,
      33,    95,   104,   106,   113,   151,   167,   121,   127,   126,
     133,   142,   135,   136,   137,   138,   123,   161,   139,     0,
     140,   152,    60,   156,     0,   157,   159,   162,   166,   169,
     168,   172,   173,   174
};

static const yytype_int16 yycheck[] =
{
      33,    27,    49,    56,    43,     4,    49,    20,    21,    37,
      38,    10,     3,    26,    42,    17,     0,    19,    20,    21,
      22,    23,    55,    70,    30,    26,    27,    28,    29,    30,
      31,    32,    33,    34,    35,    36,    37,    84,    43,    45,
      45,    39,    44,    69,    45,    43,    39,    45,    30,    43,
      97,    98,    45,    45,    97,    98,    38,    39,    40,    41,
      42,    94,    39,    45,    43,   112,    44,    46,    45,   112,
      83,   124,    26,   120,    45,   103,    30,    31,    32,    33,
      34,    35,    36,    37,    11,    12,    13,    14,    15,    44,
      43,    45,    39,    46,    30,   142,    23,   144,    45,   142,
      45,   144,    38,    39,    40,    41,    42,    44,    45,    45,
      71,    72,    73,   160,   127,   148,   163,   160,    43,    39,
     167,    43,   169,    45,   167,    45,   169,   160,   156,    25,
      26,    27,    28,    29,    30,    31,    32,    33,    34,    35,
      36,    37,     5,     6,     7,     8,     9,    45,    44,    45,
      39,    43,    44,    16,    43,    43,    45,    46,    43,    44,
      44,    46,    43,    44,    44,    46,    43,    44,    43,    44,
      43,    44,    43,    44,    43,    44,    43,    44,    44,    45,
      44,    43,    44,    99,   100,    44,    44,    25,    25,    24,
      45,    39,    43,    43,    43,    25,   163,    44,    43,    45,
      43,    45,    44,    44,    44,    44,    92,   151,    44,    -1,
      44,    44,    43,    45,    -1,    44,    44,    44,    43,    45,
      44,    44,    44,    44
};

  /* YYSTOS[STATE-NUM] -- The (internal number of the) accessing
     symbol of state STATE-NUM.  */
static const yytype_uint8 yystos[] =
{
       0,    43,    48,     3,     0,    43,     4,    10,    45,    45,
      44,    44,    49,    50,    43,    44,    51,    43,    44,     5,
       6,     7,     8,     9,    16,    11,    12,    13,    14,    15,
      23,    63,    39,    45,    60,    61,    60,    55,    56,    45,
      45,    60,    55,    43,    54,    57,    58,    59,    43,    45,
      43,    44,    45,    43,    45,    62,    39,    61,    44,    44,
      43,    44,    57,    44,    57,    52,    44,    44,    44,    25,
      26,    27,    28,    29,    30,    31,    32,    33,    34,    35,
      36,    37,    44,    45,    64,    44,    25,    43,    46,    59,
      65,    25,    24,    61,    62,    39,    17,    19,    20,    21,
      22,    23,    44,    55,    43,    59,    43,    58,    58,    58,
      60,    60,    59,    43,    30,    38,    39,    40,    41,    42,
      66,    44,    53,    63,    39,    61,    45,    43,    43,    65,
      65,    54,    54,    43,    44,    44,    44,    44,    44,    44,
      44,    65,    45,    65,    67,    43,    44,    44,    62,    60,
      30,    25,    44,    65,    44,    65,    45,    44,    61,    44,
      45,    53,    44,    57,    65,    44,    43,    67,    44,    45,
      44,    65,    44,    44,    44
};

  /* YYR1[YYN] -- Symbol number of symbol that rule YYN derives.  */
static const yytype_uint8 yyr1[] =
{
       0,    47,    48,    48,    49,    49,    49,    49,    49,    49,
      49,    50,    50,    50,    50,    50,    50,    50,    50,    51,
      52,    52,    52,    52,    52,    52,    52,    52,    53,    53,
      54,    54,    54,    54,    55,    55,    56,    56,    56,    57,
      57,    57,    57,    58,    58,    58,    59,    59,    60,    60,
      61,    61,    61,    61,    62,    62,    63,    63,    64,    64,
      64,    64,    64,    64,    64,    64,    65,    65,    65,    66,
      66,    66,    66,    66,    67,    67
};

  /* YYR2[YYN] -- Number of symbols on the right hand side of rule YYN.  */
static const yytype_uint8 yyr2[] =
{
       0,     2,     8,     8,     0,     5,     5,     5,     5,     5,
       2,     0,     5,     5,     5,     5,     8,     6,    15,     5,
       0,     5,     7,     3,     3,     3,     3,     6,     0,     6,
       0,     2,     1,     4,     0,     2,     0,     2,     4,     1,
       4,     4,     4,     1,     4,     5,     4,     4,     3,     1,
       0,     2,     4,     6,     1,     4,     0,     2,     1,     1,
       1,     1,     1,     1,     1,     1,     1,     1,     4,     1,
       1,     1,     1,     1,     1,     2
};


#define yyerrok         (yyerrstatus = 0)
#define yyclearin       (yychar = YYEMPTY)
#define YYEMPTY         (-2)
#define YYEOF           0

#define YYACCEPT        goto yyacceptlab
#define YYABORT         goto yyabortlab
#define YYERROR         goto yyerrorlab


#define YYRECOVERING()  (!!yyerrstatus)

#define YYBACKUP(Token, Value)                                  \
do                                                              \
  if (yychar == YYEMPTY)                                        \
    {                                                           \
      yychar = (Token);                                         \
      yylval = (Value);                                         \
      YYPOPSTACK (yylen);                                       \
      yystate = *yyssp;                                         \
      goto yybackup;                                            \
    }                                                           \
  else                                                          \
    {                                                           \
      yyerror (domain, YY_("syntax error: cannot back up")); \
      YYERROR;                                                  \
    }                                                           \
while (0)

/* Error token number */
#define YYTERROR        1
#define YYERRCODE       256



/* Enable debugging if requested.  */
#if YYDEBUG

# ifndef YYFPRINTF
#  include <stdio.h> /* INFRINGES ON USER NAME SPACE */
#  define YYFPRINTF fprintf
# endif

# define YYDPRINTF(Args)                        \
do {                                            \
  if (yydebug)                                  \
    YYFPRINTF Args;                             \
} while (0)

/* This macro is provided for backward compatibility. */
#ifndef YY_LOCATION_PRINT
# define YY_LOCATION_PRINT(File, Loc) ((void) 0)
#endif


# define YY_SYMBOL_PRINT(Title, Type, Value, Location)                    \
do {                                                                      \
  if (yydebug)                                                            \
    {                                                                     \
      YYFPRINTF (stderr, "%s ", Title);                                   \
      yy_symbol_print (stderr,                                            \
                  Type, Value, domain); \
      YYFPRINTF (stderr, "\n");                                           \
    }                                                                     \
} while (0)


/*----------------------------------------.
| Print this symbol's value on YYOUTPUT.  |
`----------------------------------------*/

static void
yy_symbol_value_print (FILE *yyoutput, int yytype, YYSTYPE const * const yyvaluep, PDDLDomain *domain)
{
  FILE *yyo = yyoutput;
  YYUSE (yyo);
  YYUSE (domain);
  if (!yyvaluep)
    return;
# ifdef YYPRINT
  if (yytype < YYNTOKENS)
    YYPRINT (yyoutput, yytoknum[yytype], *yyvaluep);
# endif
  YYUSE (yytype);
}


/*--------------------------------.
| Print this symbol on YYOUTPUT.  |
`--------------------------------*/

static void
yy_symbol_print (FILE *yyoutput, int yytype, YYSTYPE const * const yyvaluep, PDDLDomain *domain)
{
  YYFPRINTF (yyoutput, "%s %s (",
             yytype < YYNTOKENS ? "token" : "nterm", yytname[yytype]);

  yy_symbol_value_print (yyoutput, yytype, yyvaluep, domain);
  YYFPRINTF (yyoutput, ")");
}

/*------------------------------------------------------------------.
| yy_stack_print -- Print the state stack from its BOTTOM up to its |
| TOP (included).                                                   |
`------------------------------------------------------------------*/

static void
yy_stack_print (yytype_int16 *yybottom, yytype_int16 *yytop)
{
  YYFPRINTF (stderr, "Stack now");
  for (; yybottom <= yytop; yybottom++)
    {
      int yybot = *yybottom;
      YYFPRINTF (stderr, " %d", yybot);
    }
  YYFPRINTF (stderr, "\n");
}

# define YY_STACK_PRINT(Bottom, Top)                            \
do {                                                            \
  if (yydebug)                                                  \
    yy_stack_print ((Bottom), (Top));                           \
} while (0)


/*------------------------------------------------.
| Report that the YYRULE is going to be reduced.  |
`------------------------------------------------*/

static void
yy_reduce_print (yytype_int16 *yyssp, YYSTYPE *yyvsp, int yyrule, PDDLDomain *domain)
{
  unsigned long int yylno = yyrline[yyrule];
  int yynrhs = yyr2[yyrule];
  int yyi;
  YYFPRINTF (stderr, "Reducing stack by rule %d (line %lu):\n",
             yyrule - 1, yylno);
  /* The symbols being reduced.  */
  for (yyi = 0; yyi < yynrhs; yyi++)
    {
      YYFPRINTF (stderr, "   $%d = ", yyi + 1);
      yy_symbol_print (stderr,
                       yystos[yyssp[yyi + 1 - yynrhs]],
                       &(yyvsp[(yyi + 1) - (yynrhs)])
                                              , domain);
      YYFPRINTF (stderr, "\n");
    }
}

# define YY_REDUCE_PRINT(Rule)          \
do {                                    \
  if (yydebug)                          \
    yy_reduce_print (yyssp, yyvsp, Rule, domain); \
} while (0)

/* Nonzero means print parse trace.  It is left uninitialized so that
   multiple parsers can coexist.  */
int yydebug;
#else /* !YYDEBUG */
# define YYDPRINTF(Args)
# define YY_SYMBOL_PRINT(Title, Type, Value, Location)
# define YY_STACK_PRINT(Bottom, Top)
# define YY_REDUCE_PRINT(Rule)
#endif /* !YYDEBUG */


/* YYINITDEPTH -- initial size of the parser's stacks.  */
#ifndef YYINITDEPTH
# define YYINITDEPTH 200
#endif

/* YYMAXDEPTH -- maximum size the stacks can grow to (effective only
   if the built-in stack extension method is used).

   Do not make this value too large; the results are undefined if
   YYSTACK_ALLOC_MAXIMUM < YYSTACK_BYTES (YYMAXDEPTH)
   evaluated with infinite-precision integer arithmetic.  */

#ifndef YYMAXDEPTH
# define YYMAXDEPTH 10000
#endif


#if YYERROR_VERBOSE

# ifndef yystrlen
#  if defined __GLIBC__ && defined _STRING_H
#   define yystrlen strlen
#  else
/* Return the length of YYSTR.  */
static YYSIZE_T
yystrlen (const char *yystr)
{
  YYSIZE_T yylen;
  for (yylen = 0; yystr[yylen]; yylen++)
    continue;
  return yylen;
}
#  endif
# endif

# ifndef yystpcpy
#  if defined __GLIBC__ && defined _STRING_H && defined _GNU_SOURCE
#   define yystpcpy stpcpy
#  else
/* Copy YYSRC to YYDEST, returning the address of the terminating '\0' in
   YYDEST.  */
static char *
yystpcpy (char *yydest, const char *yysrc)
{
  char *yyd = yydest;
  const char *yys = yysrc;

  while ((*yyd++ = *yys++) != '\0')
    continue;

  return yyd - 1;
}
#  endif
# endif

# ifndef yytnamerr
/* Copy to YYRES the contents of YYSTR after stripping away unnecessary
   quotes and backslashes, so that it's suitable for yyerror.  The
   heuristic is that double-quoting is unnecessary unless the string
   contains an apostrophe, a comma, or backslash (other than
   backslash-backslash).  YYSTR is taken from yytname.  If YYRES is
   null, do not copy; instead, return the length of what the result
   would have been.  */
static YYSIZE_T
yytnamerr (char *yyres, const char *yystr)
{
  if (*yystr == '"')
    {
      YYSIZE_T yyn = 0;
      char const *yyp = yystr;

      for (;;)
        switch (*++yyp)
          {
          case '\'':
          case ',':
            goto do_not_strip_quotes;

          case '\\':
            if (*++yyp != '\\')
              goto do_not_strip_quotes;
            /* Fall through.  */
          default:
            if (yyres)
              yyres[yyn] = *yyp;
            yyn++;
            break;

          case '"':
            if (yyres)
              yyres[yyn] = '\0';
            return yyn;
          }
    do_not_strip_quotes: ;
    }

  if (! yyres)
    return yystrlen (yystr);

  return yystpcpy (yyres, yystr) - yyres;
}
# endif

/* Copy into *YYMSG, which is of size *YYMSG_ALLOC, an error message
   about the unexpected token YYTOKEN for the state stack whose top is
   YYSSP.

   Return 0 if *YYMSG was successfully written.  Return 1 if *YYMSG is
   not large enough to hold the message.  In that case, also set
   *YYMSG_ALLOC to the required number of bytes.  Return 2 if the
   required number of bytes is too large to store.  */
static int
yysyntax_error (YYSIZE_T *yymsg_alloc, char **yymsg,
                yytype_int16 *yyssp, int yytoken)
{
  YYSIZE_T yysize0 = yytnamerr (YY_NULLPTR, yytname[yytoken]);
  YYSIZE_T yysize = yysize0;
  enum { YYERROR_VERBOSE_ARGS_MAXIMUM = 5 };
  /* Internationalized format string. */
  const char *yyformat = YY_NULLPTR;
  /* Arguments of yyformat. */
  char const *yyarg[YYERROR_VERBOSE_ARGS_MAXIMUM];
  /* Number of reported tokens (one for the "unexpected", one per
     "expected"). */
  int yycount = 0;

  /* There are many possibilities here to consider:
     - If this state is a consistent state with a default action, then
       the only way this function was invoked is if the default action
       is an error action.  In that case, don't check for expected
       tokens because there are none.
     - The only way there can be no lookahead present (in yychar) is if
       this state is a consistent state with a default action.  Thus,
       detecting the absence of a lookahead is sufficient to determine
       that there is no unexpected or expected token to report.  In that
       case, just report a simple "syntax error".
     - Don't assume there isn't a lookahead just because this state is a
       consistent state with a default action.  There might have been a
       previous inconsistent state, consistent state with a non-default
       action, or user semantic action that manipulated yychar.
     - Of course, the expected token list depends on states to have
       correct lookahead information, and it depends on the parser not
       to perform extra reductions after fetching a lookahead from the
       scanner and before detecting a syntax error.  Thus, state merging
       (from LALR or IELR) and default reductions corrupt the expected
       token list.  However, the list is correct for canonical LR with
       one exception: it will still contain any token that will not be
       accepted due to an error action in a later state.
  */
  if (yytoken != YYEMPTY)
    {
      int yyn = yypact[*yyssp];
      yyarg[yycount++] = yytname[yytoken];
      if (!yypact_value_is_default (yyn))
        {
          /* Start YYX at -YYN if negative to avoid negative indexes in
             YYCHECK.  In other words, skip the first -YYN actions for
             this state because they are default actions.  */
          int yyxbegin = yyn < 0 ? -yyn : 0;
          /* Stay within bounds of both yycheck and yytname.  */
          int yychecklim = YYLAST - yyn + 1;
          int yyxend = yychecklim < YYNTOKENS ? yychecklim : YYNTOKENS;
          int yyx;

          for (yyx = yyxbegin; yyx < yyxend; ++yyx)
            if (yycheck[yyx + yyn] == yyx && yyx != YYTERROR
                && !yytable_value_is_error (yytable[yyx + yyn]))
              {
                if (yycount == YYERROR_VERBOSE_ARGS_MAXIMUM)
                  {
                    yycount = 1;
                    yysize = yysize0;
                    break;
                  }
                yyarg[yycount++] = yytname[yyx];
                {
                  YYSIZE_T yysize1 = yysize + yytnamerr (YY_NULLPTR, yytname[yyx]);
                  if (! (yysize <= yysize1
                         && yysize1 <= YYSTACK_ALLOC_MAXIMUM))
                    return 2;
                  yysize = yysize1;
                }
              }
        }
    }

  switch (yycount)
    {
# define YYCASE_(N, S)                      \
      case N:                               \
        yyformat = S;                       \
      break
      YYCASE_(0, YY_("syntax error"));
      YYCASE_(1, YY_("syntax error, unexpected %s"));
      YYCASE_(2, YY_("syntax error, unexpected %s, expecting %s"));
      YYCASE_(3, YY_("syntax error, unexpected %s, expecting %s or %s"));
      YYCASE_(4, YY_("syntax error, unexpected %s, expecting %s or %s or %s"));
      YYCASE_(5, YY_("syntax error, unexpected %s, expecting %s or %s or %s or %s"));
# undef YYCASE_
    }

  {
    YYSIZE_T yysize1 = yysize + yystrlen (yyformat);
    if (! (yysize <= yysize1 && yysize1 <= YYSTACK_ALLOC_MAXIMUM))
      return 2;
    yysize = yysize1;
  }

  if (*yymsg_alloc < yysize)
    {
      *yymsg_alloc = 2 * yysize;
      if (! (yysize <= *yymsg_alloc
             && *yymsg_alloc <= YYSTACK_ALLOC_MAXIMUM))
        *yymsg_alloc = YYSTACK_ALLOC_MAXIMUM;
      return 1;
    }

  /* Avoid sprintf, as that infringes on the user's name space.
     Don't have undefined behavior even if the translation
     produced a string with the wrong number of "%s"s.  */
  {
    char *yyp = *yymsg;
    int yyi = 0;
    while ((*yyp = *yyformat) != '\0')
      if (*yyp == '%' && yyformat[1] == 's' && yyi < yycount)
        {
          yyp += yytnamerr (yyp, yyarg[yyi++]);
          yyformat += 2;
        }
      else
        {
          yyp++;
          yyformat++;
        }
  }
  return 0;
}
#endif /* YYERROR_VERBOSE */

/*-----------------------------------------------.
| Release the memory associated to this symbol.  |
`-----------------------------------------------*/

static void
yydestruct (const char *yymsg, int yytype, YYSTYPE *yyvaluep, PDDLDomain *domain)
{
  YYUSE (yyvaluep);
  YYUSE (domain);
  if (!yymsg)
    yymsg = "Deleting";
  YY_SYMBOL_PRINT (yymsg, yytype, yyvaluep, yylocationp);

  YY_IGNORE_MAYBE_UNINITIALIZED_BEGIN
  YYUSE (yytype);
  YY_IGNORE_MAYBE_UNINITIALIZED_END
}




/* The lookahead symbol.  */
int yychar;

/* The semantic value of the lookahead symbol.  */
YYSTYPE yylval;
/* Number of syntax errors so far.  */
int yynerrs;


/*----------.
| yyparse.  |
`----------*/

int
yyparse (PDDLDomain *domain)
{
    int yystate;
    /* Number of tokens to shift before error messages enabled.  */
    int yyerrstatus;

    /* The stacks and their tools:
       'yyss': related to states.
       'yyvs': related to semantic values.

       Refer to the stacks through separate pointers, to allow yyoverflow
       to reallocate them elsewhere.  */

    /* The state stack.  */
    yytype_int16 yyssa[YYINITDEPTH];
    yytype_int16 *yyss;
    yytype_int16 *yyssp;

    /* The semantic value stack.  */
    YYSTYPE yyvsa[YYINITDEPTH];
    YYSTYPE *yyvs;
    YYSTYPE *yyvsp;

    YYSIZE_T yystacksize;

  int yyn;
  int yyresult;
  /* Lookahead token as an internal (translated) token number.  */
  int yytoken = 0;
  /* The variables used to return semantic value and location from the
     action routines.  */
  YYSTYPE yyval;

#if YYERROR_VERBOSE
  /* Buffer for error messages, and its allocated size.  */
  char yymsgbuf[128];
  char *yymsg = yymsgbuf;
  YYSIZE_T yymsg_alloc = sizeof yymsgbuf;
#endif

#define YYPOPSTACK(N)   (yyvsp -= (N), yyssp -= (N))

  /* The number of symbols on the RHS of the reduced rule.
     Keep to zero when no symbol should be popped.  */
  int yylen = 0;

  yyssp = yyss = yyssa;
  yyvsp = yyvs = yyvsa;
  yystacksize = YYINITDEPTH;

  YYDPRINTF ((stderr, "Starting parse\n"));

  yystate = 0;
  yyerrstatus = 0;
  yynerrs = 0;
  yychar = YYEMPTY; /* Cause a token to be read.  */
  goto yysetstate;

/*------------------------------------------------------------.
| yynewstate -- Push a new state, which is found in yystate.  |
`------------------------------------------------------------*/
 yynewstate:
  /* In all cases, when you get here, the value and location stacks
     have just been pushed.  So pushing a state here evens the stacks.  */
  yyssp++;

 yysetstate:
  *yyssp = yystate;

  if (yyss + yystacksize - 1 <= yyssp)
    {
      /* Get the current used size of the three stacks, in elements.  */
      YYSIZE_T yysize = yyssp - yyss + 1;

#ifdef yyoverflow
      {
        /* Give user a chance to reallocate the stack.  Use copies of
           these so that the &'s don't force the real ones into
           memory.  */
        YYSTYPE *yyvs1 = yyvs;
        yytype_int16 *yyss1 = yyss;

        /* Each stack pointer address is followed by the size of the
           data in use in that stack, in bytes.  This used to be a
           conditional around just the two extra args, but that might
           be undefined if yyoverflow is a macro.  */
        yyoverflow (YY_("memory exhausted"),
                    &yyss1, yysize * sizeof (*yyssp),
                    &yyvs1, yysize * sizeof (*yyvsp),
                    &yystacksize);

        yyss = yyss1;
        yyvs = yyvs1;
      }
#else /* no yyoverflow */
# ifndef YYSTACK_RELOCATE
      goto yyexhaustedlab;
# else
      /* Extend the stack our own way.  */
      if (YYMAXDEPTH <= yystacksize)
        goto yyexhaustedlab;
      yystacksize *= 2;
      if (YYMAXDEPTH < yystacksize)
        yystacksize = YYMAXDEPTH;

      {
        yytype_int16 *yyss1 = yyss;
        union yyalloc *yyptr =
          (union yyalloc *) YYSTACK_ALLOC (YYSTACK_BYTES (yystacksize));
        if (! yyptr)
          goto yyexhaustedlab;
        YYSTACK_RELOCATE (yyss_alloc, yyss);
        YYSTACK_RELOCATE (yyvs_alloc, yyvs);
#  undef YYSTACK_RELOCATE
        if (yyss1 != yyssa)
          YYSTACK_FREE (yyss1);
      }
# endif
#endif /* no yyoverflow */

      yyssp = yyss + yysize - 1;
      yyvsp = yyvs + yysize - 1;

      YYDPRINTF ((stderr, "Stack size increased to %lu\n",
                  (unsigned long int) yystacksize));

      if (yyss + yystacksize - 1 <= yyssp)
        YYABORT;
    }

  YYDPRINTF ((stderr, "Entering state %d\n", yystate));

  if (yystate == YYFINAL)
    YYACCEPT;

  goto yybackup;

/*-----------.
| yybackup.  |
`-----------*/
yybackup:

  /* Do appropriate processing given the current state.  Read a
     lookahead token if we need one and don't already have one.  */

  /* First try to decide what to do without reference to lookahead token.  */
  yyn = yypact[yystate];
  if (yypact_value_is_default (yyn))
    goto yydefault;

  /* Not known => get a lookahead token if don't already have one.  */

  /* YYCHAR is either YYEMPTY or YYEOF or a valid lookahead symbol.  */
  if (yychar == YYEMPTY)
    {
      YYDPRINTF ((stderr, "Reading a token: "));
      yychar = yylex (domain);
    }

  if (yychar <= YYEOF)
    {
      yychar = yytoken = YYEOF;
      YYDPRINTF ((stderr, "Now at end of input.\n"));
    }
  else
    {
      yytoken = YYTRANSLATE (yychar);
      YY_SYMBOL_PRINT ("Next token is", yytoken, &yylval, &yylloc);
    }

  /* If the proper action on seeing token YYTOKEN is to reduce or to
     detect an error, take that action.  */
  yyn += yytoken;
  if (yyn < 0 || YYLAST < yyn || yycheck[yyn] != yytoken)
    goto yydefault;
  yyn = yytable[yyn];
  if (yyn <= 0)
    {
      if (yytable_value_is_error (yyn))
        goto yyerrlab;
      yyn = -yyn;
      goto yyreduce;
    }

  /* Count tokens shifted since error; after three, turn off error
     status.  */
  if (yyerrstatus)
    yyerrstatus--;

  /* Shift the lookahead token.  */
  YY_SYMBOL_PRINT ("Shifting", yytoken, &yylval, &yylloc);

  /* Discard the shifted token.  */
  yychar = YYEMPTY;

  yystate = yyn;
  YY_IGNORE_MAYBE_UNINITIALIZED_BEGIN
  *++yyvsp = yylval;
  YY_IGNORE_MAYBE_UNINITIALIZED_END

  goto yynewstate;


/*-----------------------------------------------------------.
| yydefault -- do the default action for the current state.  |
`-----------------------------------------------------------*/
yydefault:
  yyn = yydefact[yystate];
  if (yyn == 0)
    goto yyerrlab;
  goto yyreduce;


/*-----------------------------.
| yyreduce -- Do a reduction.  |
`-----------------------------*/
yyreduce:
  /* yyn is the number of a rule to reduce with.  */
  yylen = yyr2[yyn];

  /* If YYLEN is nonzero, implement the default value of the action:
     '$$ = $1'.

     Otherwise, the following line sets YYVAL to garbage.
     This behavior is undocumented and Bison
     users should not rely upon it.  Assigning to YYVAL
     unconditionally makes the parser a bit smaller, and it avoids a
     GCC warning that YYVAL may be used uninitialized.  */
  yyval = yyvsp[1-yylen];


  YY_REDUCE_PRINT (yyn);
  switch (yyn)
    {
        case 2:
#line 56 "parser.y" /* yacc.c:1646  */
    { domain->name = (yyvsp[-3].string); }
#line 1409 "/home/john/Desktop/PDDL/planner/descarwin/build/cpt-yahsp/src/parser.tab.cpp" /* yacc.c:1646  */
    break;

  case 3:
#line 57 "parser.y" /* yacc.c:1646  */
    { domain->probname = (yyvsp[-3].string); }
#line 1415 "/home/john/Desktop/PDDL/planner/descarwin/build/cpt-yahsp/src/parser.tab.cpp" /* yacc.c:1646  */
    break;

  case 5:
#line 61 "parser.y" /* yacc.c:1646  */
    { domain->token_requirements = (yyvsp[-1].list); }
#line 1421 "/home/john/Desktop/PDDL/planner/descarwin/build/cpt-yahsp/src/parser.tab.cpp" /* yacc.c:1646  */
    break;

  case 6:
#line 62 "parser.y" /* yacc.c:1646  */
    { domain->token_types = (yyvsp[-1].list); }
#line 1427 "/home/john/Desktop/PDDL/planner/descarwin/build/cpt-yahsp/src/parser.tab.cpp" /* yacc.c:1646  */
    break;

  case 7:
#line 63 "parser.y" /* yacc.c:1646  */
    { domain->token_constants = (yyvsp[-1].list); }
#line 1433 "/home/john/Desktop/PDDL/planner/descarwin/build/cpt-yahsp/src/parser.tab.cpp" /* yacc.c:1646  */
    break;

  case 8:
#line 64 "parser.y" /* yacc.c:1646  */
    { domain->token_predicates = (yyvsp[-1].list); }
#line 1439 "/home/john/Desktop/PDDL/planner/descarwin/build/cpt-yahsp/src/parser.tab.cpp" /* yacc.c:1646  */
    break;

  case 9:
#line 65 "parser.y" /* yacc.c:1646  */
    { domain->token_functions = (yyvsp[-1].list); }
#line 1445 "/home/john/Desktop/PDDL/planner/descarwin/build/cpt-yahsp/src/parser.tab.cpp" /* yacc.c:1646  */
    break;

  case 10:
#line 66 "parser.y" /* yacc.c:1646  */
    { domain_add_token_operator(domain, (yyvsp[0].ope)); }
#line 1451 "/home/john/Desktop/PDDL/planner/descarwin/build/cpt-yahsp/src/parser.tab.cpp" /* yacc.c:1646  */
    break;

  case 12:
#line 70 "parser.y" /* yacc.c:1646  */
    { if (strcmp(domain->name, (yyvsp[-1].string)) != 0) error(parser, "Domain name '%s' and problem name '%s' do not correspond", domain->name, (yyvsp[-1].string)); }
#line 1457 "/home/john/Desktop/PDDL/planner/descarwin/build/cpt-yahsp/src/parser.tab.cpp" /* yacc.c:1646  */
    break;

  case 13:
#line 71 "parser.y" /* yacc.c:1646  */
    { domain->token_objects = (yyvsp[-1].list); }
#line 1463 "/home/john/Desktop/PDDL/planner/descarwin/build/cpt-yahsp/src/parser.tab.cpp" /* yacc.c:1646  */
    break;

  case 14:
#line 72 "parser.y" /* yacc.c:1646  */
    { domain->token_init = (yyvsp[-1].list); }
#line 1469 "/home/john/Desktop/PDDL/planner/descarwin/build/cpt-yahsp/src/parser.tab.cpp" /* yacc.c:1646  */
    break;

  case 15:
#line 73 "parser.y" /* yacc.c:1646  */
    { domain->token_goal = (yyvsp[-1].list); }
#line 1475 "/home/john/Desktop/PDDL/planner/descarwin/build/cpt-yahsp/src/parser.tab.cpp" /* yacc.c:1646  */
    break;

  case 16:
#line 74 "parser.y" /* yacc.c:1646  */
    { domain->token_ac_constraints = (yyvsp[-2].list); }
#line 1481 "/home/john/Desktop/PDDL/planner/descarwin/build/cpt-yahsp/src/parser.tab.cpp" /* yacc.c:1646  */
    break;

  case 17:
#line 75 "parser.y" /* yacc.c:1646  */
    { domain->token_metric = (yyvsp[-1].list); domain->metric_function = (yyvsp[-2].string); }
#line 1487 "/home/john/Desktop/PDDL/planner/descarwin/build/cpt-yahsp/src/parser.tab.cpp" /* yacc.c:1646  */
    break;

  case 18:
#line 77 "parser.y" /* yacc.c:1646  */
    { domain->token_metric = (yyvsp[-7].list); domain->metric_function = (yyvsp[-8].string); }
#line 1493 "/home/john/Desktop/PDDL/planner/descarwin/build/cpt-yahsp/src/parser.tab.cpp" /* yacc.c:1646  */
    break;

  case 19:
#line 82 "parser.y" /* yacc.c:1646  */
    { ((yyval.ope) = (yyvsp[-1].ope))->name = (yyvsp[-2].string); }
#line 1499 "/home/john/Desktop/PDDL/planner/descarwin/build/cpt-yahsp/src/parser.tab.cpp" /* yacc.c:1646  */
    break;

  case 20:
#line 86 "parser.y" /* yacc.c:1646  */
    { cpt_calloc((yyval.ope), 1); }
#line 1505 "/home/john/Desktop/PDDL/planner/descarwin/build/cpt-yahsp/src/parser.tab.cpp" /* yacc.c:1646  */
    break;

  case 21:
#line 87 "parser.y" /* yacc.c:1646  */
    { ((yyval.ope) = (yyvsp[-4].ope))->token_parameters = (yyvsp[-1].list); }
#line 1511 "/home/john/Desktop/PDDL/planner/descarwin/build/cpt-yahsp/src/parser.tab.cpp" /* yacc.c:1646  */
    break;

  case 22:
#line 88 "parser.y" /* yacc.c:1646  */
    { ((yyval.ope) = (yyvsp[-6].ope))->token_duration = (yyvsp[-1].list); }
#line 1517 "/home/john/Desktop/PDDL/planner/descarwin/build/cpt-yahsp/src/parser.tab.cpp" /* yacc.c:1646  */
    break;

  case 23:
#line 89 "parser.y" /* yacc.c:1646  */
    { ((yyval.ope) = (yyvsp[-2].ope))->token_duration = (yyvsp[0].list); }
#line 1523 "/home/john/Desktop/PDDL/planner/descarwin/build/cpt-yahsp/src/parser.tab.cpp" /* yacc.c:1646  */
    break;

  case 24:
#line 90 "parser.y" /* yacc.c:1646  */
    { ((yyval.ope) = (yyvsp[-2].ope))->token_real_duration = (yyvsp[0].list); }
#line 1529 "/home/john/Desktop/PDDL/planner/descarwin/build/cpt-yahsp/src/parser.tab.cpp" /* yacc.c:1646  */
    break;

  case 25:
#line 91 "parser.y" /* yacc.c:1646  */
    { ((yyval.ope) = (yyvsp[-2].ope))->token_precondition = (yyvsp[0].list); }
#line 1535 "/home/john/Desktop/PDDL/planner/descarwin/build/cpt-yahsp/src/parser.tab.cpp" /* yacc.c:1646  */
    break;

  case 26:
#line 92 "parser.y" /* yacc.c:1646  */
    { ((yyval.ope) = (yyvsp[-2].ope))->token_effect = (yyvsp[0].list); }
#line 1541 "/home/john/Desktop/PDDL/planner/descarwin/build/cpt-yahsp/src/parser.tab.cpp" /* yacc.c:1646  */
    break;

  case 27:
#line 93 "parser.y" /* yacc.c:1646  */
    { ((yyval.ope) = (yyvsp[-5].ope))->token_ac_constraints = (yyvsp[-1].list); }
#line 1547 "/home/john/Desktop/PDDL/planner/descarwin/build/cpt-yahsp/src/parser.tab.cpp" /* yacc.c:1646  */
    break;

  case 28:
#line 97 "parser.y" /* yacc.c:1646  */
    { (yyval.list) = NULL; }
#line 1553 "/home/john/Desktop/PDDL/planner/descarwin/build/cpt-yahsp/src/parser.tab.cpp" /* yacc.c:1646  */
    break;

  case 29:
#line 98 "parser.y" /* yacc.c:1646  */
    { (yyval.list) = token_add_tail(token_add_tail((yyvsp[-5].list), NULL, (yyvsp[-2].list)), NULL, (yyvsp[-1].list)); }
#line 1559 "/home/john/Desktop/PDDL/planner/descarwin/build/cpt-yahsp/src/parser.tab.cpp" /* yacc.c:1646  */
    break;

  case 30:
#line 102 "parser.y" /* yacc.c:1646  */
    { (yyval.list) = NULL; }
#line 1565 "/home/john/Desktop/PDDL/planner/descarwin/build/cpt-yahsp/src/parser.tab.cpp" /* yacc.c:1646  */
    break;

  case 31:
#line 103 "parser.y" /* yacc.c:1646  */
    { (yyval.list) = NULL; }
#line 1571 "/home/john/Desktop/PDDL/planner/descarwin/build/cpt-yahsp/src/parser.tab.cpp" /* yacc.c:1646  */
    break;

  case 32:
#line 104 "parser.y" /* yacc.c:1646  */
    { (yyval.list) = token_add_head(NULL, NULL, (yyvsp[0].list)); }
#line 1577 "/home/john/Desktop/PDDL/planner/descarwin/build/cpt-yahsp/src/parser.tab.cpp" /* yacc.c:1646  */
    break;

  case 33:
#line 105 "parser.y" /* yacc.c:1646  */
    { (yyval.list) = (yyvsp[-1].list); }
#line 1583 "/home/john/Desktop/PDDL/planner/descarwin/build/cpt-yahsp/src/parser.tab.cpp" /* yacc.c:1646  */
    break;

  case 34:
#line 109 "parser.y" /* yacc.c:1646  */
    { (yyval.list) = NULL; }
#line 1589 "/home/john/Desktop/PDDL/planner/descarwin/build/cpt-yahsp/src/parser.tab.cpp" /* yacc.c:1646  */
    break;

  case 35:
#line 110 "parser.y" /* yacc.c:1646  */
    { (yyval.list) = token_add_tail((yyvsp[-1].list), NULL, (yyvsp[0].list)); }
#line 1595 "/home/john/Desktop/PDDL/planner/descarwin/build/cpt-yahsp/src/parser.tab.cpp" /* yacc.c:1646  */
    break;

  case 36:
#line 114 "parser.y" /* yacc.c:1646  */
    { (yyval.list) = NULL; }
#line 1601 "/home/john/Desktop/PDDL/planner/descarwin/build/cpt-yahsp/src/parser.tab.cpp" /* yacc.c:1646  */
    break;

  case 37:
#line 115 "parser.y" /* yacc.c:1646  */
    { (yyval.list) = token_add_tail((yyvsp[-1].list), NULL, (yyvsp[0].list)); }
#line 1607 "/home/john/Desktop/PDDL/planner/descarwin/build/cpt-yahsp/src/parser.tab.cpp" /* yacc.c:1646  */
    break;

  case 38:
#line 116 "parser.y" /* yacc.c:1646  */
    { (yyval.list) = token_add_tail((yyvsp[-3].list), NULL, (yyvsp[-2].list)); }
#line 1613 "/home/john/Desktop/PDDL/planner/descarwin/build/cpt-yahsp/src/parser.tab.cpp" /* yacc.c:1646  */
    break;

  case 39:
#line 120 "parser.y" /* yacc.c:1646  */
    { (yyval.list) = (yyvsp[0].list); }
#line 1619 "/home/john/Desktop/PDDL/planner/descarwin/build/cpt-yahsp/src/parser.tab.cpp" /* yacc.c:1646  */
    break;

  case 40:
#line 121 "parser.y" /* yacc.c:1646  */
    { (yyval.list) = token_set_temporal_mod((yyvsp[-1].list), ATSTART_MOD); }
#line 1625 "/home/john/Desktop/PDDL/planner/descarwin/build/cpt-yahsp/src/parser.tab.cpp" /* yacc.c:1646  */
    break;

  case 41:
#line 122 "parser.y" /* yacc.c:1646  */
    { (yyval.list) = token_set_temporal_mod((yyvsp[-1].list), ATEND_MOD); }
#line 1631 "/home/john/Desktop/PDDL/planner/descarwin/build/cpt-yahsp/src/parser.tab.cpp" /* yacc.c:1646  */
    break;

  case 42:
#line 123 "parser.y" /* yacc.c:1646  */
    { (yyval.list) = token_set_temporal_mod((yyvsp[-1].list), OVERALL_MOD); }
#line 1637 "/home/john/Desktop/PDDL/planner/descarwin/build/cpt-yahsp/src/parser.tab.cpp" /* yacc.c:1646  */
    break;

  case 43:
#line 127 "parser.y" /* yacc.c:1646  */
    { (yyval.list) = token_set_sign_mod((yyvsp[0].list), POS_MOD); }
#line 1643 "/home/john/Desktop/PDDL/planner/descarwin/build/cpt-yahsp/src/parser.tab.cpp" /* yacc.c:1646  */
    break;

  case 44:
#line 128 "parser.y" /* yacc.c:1646  */
    { (yyval.list) = token_set_sign_mod((yyvsp[-1].list), NEG_MOD); }
#line 1649 "/home/john/Desktop/PDDL/planner/descarwin/build/cpt-yahsp/src/parser.tab.cpp" /* yacc.c:1646  */
    break;

  case 45:
#line 129 "parser.y" /* yacc.c:1646  */
    { (yyval.list) = token_set_function_mod(token_set_sub((yyvsp[-2].list), (yyvsp[-1].list)), (FunctionModality) (yyvsp[-3].number)); }
#line 1655 "/home/john/Desktop/PDDL/planner/descarwin/build/cpt-yahsp/src/parser.tab.cpp" /* yacc.c:1646  */
    break;

  case 46:
#line 133 "parser.y" /* yacc.c:1646  */
    { (yyval.list) = token_add_head((yyvsp[-1].list), (yyvsp[-2].string), NULL); }
#line 1661 "/home/john/Desktop/PDDL/planner/descarwin/build/cpt-yahsp/src/parser.tab.cpp" /* yacc.c:1646  */
    break;

  case 47:
#line 134 "parser.y" /* yacc.c:1646  */
    { (yyval.list) = token_set_equality_mod(token_add_head((yyvsp[-1].list), "=", NULL), EQUAL_MOD); }
#line 1667 "/home/john/Desktop/PDDL/planner/descarwin/build/cpt-yahsp/src/parser.tab.cpp" /* yacc.c:1646  */
    break;

  case 48:
#line 145 "parser.y" /* yacc.c:1646  */
    { (yyval.list) = (yyvsp[0].list); }
#line 1673 "/home/john/Desktop/PDDL/planner/descarwin/build/cpt-yahsp/src/parser.tab.cpp" /* yacc.c:1646  */
    break;

  case 49:
#line 146 "parser.y" /* yacc.c:1646  */
    { (yyval.list) = (yyvsp[0].list); }
#line 1679 "/home/john/Desktop/PDDL/planner/descarwin/build/cpt-yahsp/src/parser.tab.cpp" /* yacc.c:1646  */
    break;

  case 50:
#line 150 "parser.y" /* yacc.c:1646  */
    { (yyval.list) = NULL; }
#line 1685 "/home/john/Desktop/PDDL/planner/descarwin/build/cpt-yahsp/src/parser.tab.cpp" /* yacc.c:1646  */
    break;

  case 51:
#line 151 "parser.y" /* yacc.c:1646  */
    { (yyval.list) = token_add_head((yyvsp[0].list), (yyvsp[-1].string), NULL); }
#line 1691 "/home/john/Desktop/PDDL/planner/descarwin/build/cpt-yahsp/src/parser.tab.cpp" /* yacc.c:1646  */
    break;

  case 52:
#line 152 "parser.y" /* yacc.c:1646  */
    { (yyval.list) = token_add_head((yyvsp[0].list), (yyvsp[-3].string), (yyvsp[-1].list)); }
#line 1697 "/home/john/Desktop/PDDL/planner/descarwin/build/cpt-yahsp/src/parser.tab.cpp" /* yacc.c:1646  */
    break;

  case 53:
#line 153 "parser.y" /* yacc.c:1646  */
    { (yyval.list) = token_add_head((yyvsp[0].list), (yyvsp[-5].string), (yyvsp[-3].list)); }
#line 1703 "/home/john/Desktop/PDDL/planner/descarwin/build/cpt-yahsp/src/parser.tab.cpp" /* yacc.c:1646  */
    break;

  case 54:
#line 157 "parser.y" /* yacc.c:1646  */
    { (yyval.list) = token_add_head(NULL, (yyvsp[0].string), NULL); }
#line 1709 "/home/john/Desktop/PDDL/planner/descarwin/build/cpt-yahsp/src/parser.tab.cpp" /* yacc.c:1646  */
    break;

  case 55:
#line 158 "parser.y" /* yacc.c:1646  */
    { (yyval.list) = (yyvsp[-1].list); }
#line 1715 "/home/john/Desktop/PDDL/planner/descarwin/build/cpt-yahsp/src/parser.tab.cpp" /* yacc.c:1646  */
    break;

  case 56:
#line 162 "parser.y" /* yacc.c:1646  */
    { (yyval.list) = NULL; }
#line 1721 "/home/john/Desktop/PDDL/planner/descarwin/build/cpt-yahsp/src/parser.tab.cpp" /* yacc.c:1646  */
    break;

  case 57:
#line 163 "parser.y" /* yacc.c:1646  */
    { (yyval.list) = token_add_tail((yyvsp[-1].list), (yyvsp[0].string), NULL); }
#line 1727 "/home/john/Desktop/PDDL/planner/descarwin/build/cpt-yahsp/src/parser.tab.cpp" /* yacc.c:1646  */
    break;

  case 58:
#line 167 "parser.y" /* yacc.c:1646  */
    { (yyval.number) = EQ_MOD; }
#line 1733 "/home/john/Desktop/PDDL/planner/descarwin/build/cpt-yahsp/src/parser.tab.cpp" /* yacc.c:1646  */
    break;

  case 59:
#line 168 "parser.y" /* yacc.c:1646  */
    { (yyval.number) = LTE_MOD; }
#line 1739 "/home/john/Desktop/PDDL/planner/descarwin/build/cpt-yahsp/src/parser.tab.cpp" /* yacc.c:1646  */
    break;

  case 60:
#line 169 "parser.y" /* yacc.c:1646  */
    { (yyval.number) = GTE_MOD; }
#line 1745 "/home/john/Desktop/PDDL/planner/descarwin/build/cpt-yahsp/src/parser.tab.cpp" /* yacc.c:1646  */
    break;

  case 61:
#line 170 "parser.y" /* yacc.c:1646  */
    { (yyval.number) = LT_MOD; }
#line 1751 "/home/john/Desktop/PDDL/planner/descarwin/build/cpt-yahsp/src/parser.tab.cpp" /* yacc.c:1646  */
    break;

  case 62:
#line 171 "parser.y" /* yacc.c:1646  */
    { (yyval.number) = GT_MOD; }
#line 1757 "/home/john/Desktop/PDDL/planner/descarwin/build/cpt-yahsp/src/parser.tab.cpp" /* yacc.c:1646  */
    break;

  case 63:
#line 172 "parser.y" /* yacc.c:1646  */
    { (yyval.number) = INC_MOD; }
#line 1763 "/home/john/Desktop/PDDL/planner/descarwin/build/cpt-yahsp/src/parser.tab.cpp" /* yacc.c:1646  */
    break;

  case 64:
#line 173 "parser.y" /* yacc.c:1646  */
    { (yyval.number) = DEC_MOD; }
#line 1769 "/home/john/Desktop/PDDL/planner/descarwin/build/cpt-yahsp/src/parser.tab.cpp" /* yacc.c:1646  */
    break;

  case 65:
#line 174 "parser.y" /* yacc.c:1646  */
    { (yyval.number) = ASS_MOD; }
#line 1775 "/home/john/Desktop/PDDL/planner/descarwin/build/cpt-yahsp/src/parser.tab.cpp" /* yacc.c:1646  */
    break;

  case 66:
#line 178 "parser.y" /* yacc.c:1646  */
    { (yyval.list) = token_set_expression_mod(token_add_head(NULL, (yyvsp[0].string), NULL), NUM_MOD); }
#line 1781 "/home/john/Desktop/PDDL/planner/descarwin/build/cpt-yahsp/src/parser.tab.cpp" /* yacc.c:1646  */
    break;

  case 67:
#line 179 "parser.y" /* yacc.c:1646  */
    { (yyval.list) = token_set_expression_mod((yyvsp[0].list), ATOM_MOD); }
#line 1787 "/home/john/Desktop/PDDL/planner/descarwin/build/cpt-yahsp/src/parser.tab.cpp" /* yacc.c:1646  */
    break;

  case 68:
#line 180 "parser.y" /* yacc.c:1646  */
    { (yyval.list) = token_set_expression_mod((yyvsp[-1].list), (ExpressionModality) (yyvsp[-2].number)); }
#line 1793 "/home/john/Desktop/PDDL/planner/descarwin/build/cpt-yahsp/src/parser.tab.cpp" /* yacc.c:1646  */
    break;

  case 69:
#line 184 "parser.y" /* yacc.c:1646  */
    { (yyval.number) = ADD_MOD; }
#line 1799 "/home/john/Desktop/PDDL/planner/descarwin/build/cpt-yahsp/src/parser.tab.cpp" /* yacc.c:1646  */
    break;

  case 70:
#line 185 "parser.y" /* yacc.c:1646  */
    { (yyval.number) = SUB_MOD; }
#line 1805 "/home/john/Desktop/PDDL/planner/descarwin/build/cpt-yahsp/src/parser.tab.cpp" /* yacc.c:1646  */
    break;

  case 71:
#line 186 "parser.y" /* yacc.c:1646  */
    { (yyval.number) = MUL_MOD; }
#line 1811 "/home/john/Desktop/PDDL/planner/descarwin/build/cpt-yahsp/src/parser.tab.cpp" /* yacc.c:1646  */
    break;

  case 72:
#line 187 "parser.y" /* yacc.c:1646  */
    { (yyval.number) = DIV_MOD; }
#line 1817 "/home/john/Desktop/PDDL/planner/descarwin/build/cpt-yahsp/src/parser.tab.cpp" /* yacc.c:1646  */
    break;

  case 73:
#line 188 "parser.y" /* yacc.c:1646  */
    { (yyval.number) = FAM_MOD; }
#line 1823 "/home/john/Desktop/PDDL/planner/descarwin/build/cpt-yahsp/src/parser.tab.cpp" /* yacc.c:1646  */
    break;

  case 74:
#line 192 "parser.y" /* yacc.c:1646  */
    { (yyval.list) = token_add_head(NULL, NULL, (yyvsp[0].list)); }
#line 1829 "/home/john/Desktop/PDDL/planner/descarwin/build/cpt-yahsp/src/parser.tab.cpp" /* yacc.c:1646  */
    break;

  case 75:
#line 193 "parser.y" /* yacc.c:1646  */
    { (yyval.list) = token_add_tail((yyvsp[-1].list), NULL, (yyvsp[0].list)); }
#line 1835 "/home/john/Desktop/PDDL/planner/descarwin/build/cpt-yahsp/src/parser.tab.cpp" /* yacc.c:1646  */
    break;


#line 1839 "/home/john/Desktop/PDDL/planner/descarwin/build/cpt-yahsp/src/parser.tab.cpp" /* yacc.c:1646  */
      default: break;
    }
  /* User semantic actions sometimes alter yychar, and that requires
     that yytoken be updated with the new translation.  We take the
     approach of translating immediately before every use of yytoken.
     One alternative is translating here after every semantic action,
     but that translation would be missed if the semantic action invokes
     YYABORT, YYACCEPT, or YYERROR immediately after altering yychar or
     if it invokes YYBACKUP.  In the case of YYABORT or YYACCEPT, an
     incorrect destructor might then be invoked immediately.  In the
     case of YYERROR or YYBACKUP, subsequent parser actions might lead
     to an incorrect destructor call or verbose syntax error message
     before the lookahead is translated.  */
  YY_SYMBOL_PRINT ("-> $$ =", yyr1[yyn], &yyval, &yyloc);

  YYPOPSTACK (yylen);
  yylen = 0;
  YY_STACK_PRINT (yyss, yyssp);

  *++yyvsp = yyval;

  /* Now 'shift' the result of the reduction.  Determine what state
     that goes to, based on the state we popped back to and the rule
     number reduced by.  */

  yyn = yyr1[yyn];

  yystate = yypgoto[yyn - YYNTOKENS] + *yyssp;
  if (0 <= yystate && yystate <= YYLAST && yycheck[yystate] == *yyssp)
    yystate = yytable[yystate];
  else
    yystate = yydefgoto[yyn - YYNTOKENS];

  goto yynewstate;


/*--------------------------------------.
| yyerrlab -- here on detecting error.  |
`--------------------------------------*/
yyerrlab:
  /* Make sure we have latest lookahead translation.  See comments at
     user semantic actions for why this is necessary.  */
  yytoken = yychar == YYEMPTY ? YYEMPTY : YYTRANSLATE (yychar);

  /* If not already recovering from an error, report this error.  */
  if (!yyerrstatus)
    {
      ++yynerrs;
#if ! YYERROR_VERBOSE
      yyerror (domain, YY_("syntax error"));
#else
# define YYSYNTAX_ERROR yysyntax_error (&yymsg_alloc, &yymsg, \
                                        yyssp, yytoken)
      {
        char const *yymsgp = YY_("syntax error");
        int yysyntax_error_status;
        yysyntax_error_status = YYSYNTAX_ERROR;
        if (yysyntax_error_status == 0)
          yymsgp = yymsg;
        else if (yysyntax_error_status == 1)
          {
            if (yymsg != yymsgbuf)
              YYSTACK_FREE (yymsg);
            yymsg = (char *) YYSTACK_ALLOC (yymsg_alloc);
            if (!yymsg)
              {
                yymsg = yymsgbuf;
                yymsg_alloc = sizeof yymsgbuf;
                yysyntax_error_status = 2;
              }
            else
              {
                yysyntax_error_status = YYSYNTAX_ERROR;
                yymsgp = yymsg;
              }
          }
        yyerror (domain, yymsgp);
        if (yysyntax_error_status == 2)
          goto yyexhaustedlab;
      }
# undef YYSYNTAX_ERROR
#endif
    }



  if (yyerrstatus == 3)
    {
      /* If just tried and failed to reuse lookahead token after an
         error, discard it.  */

      if (yychar <= YYEOF)
        {
          /* Return failure if at end of input.  */
          if (yychar == YYEOF)
            YYABORT;
        }
      else
        {
          yydestruct ("Error: discarding",
                      yytoken, &yylval, domain);
          yychar = YYEMPTY;
        }
    }

  /* Else will try to reuse lookahead token after shifting the error
     token.  */
  goto yyerrlab1;


/*---------------------------------------------------.
| yyerrorlab -- error raised explicitly by YYERROR.  |
`---------------------------------------------------*/
yyerrorlab:

  /* Pacify compilers like GCC when the user code never invokes
     YYERROR and the label yyerrorlab therefore never appears in user
     code.  */
  if (/*CONSTCOND*/ 0)
     goto yyerrorlab;

  /* Do not reclaim the symbols of the rule whose action triggered
     this YYERROR.  */
  YYPOPSTACK (yylen);
  yylen = 0;
  YY_STACK_PRINT (yyss, yyssp);
  yystate = *yyssp;
  goto yyerrlab1;


/*-------------------------------------------------------------.
| yyerrlab1 -- common code for both syntax error and YYERROR.  |
`-------------------------------------------------------------*/
yyerrlab1:
  yyerrstatus = 3;      /* Each real token shifted decrements this.  */

  for (;;)
    {
      yyn = yypact[yystate];
      if (!yypact_value_is_default (yyn))
        {
          yyn += YYTERROR;
          if (0 <= yyn && yyn <= YYLAST && yycheck[yyn] == YYTERROR)
            {
              yyn = yytable[yyn];
              if (0 < yyn)
                break;
            }
        }

      /* Pop the current state because it cannot handle the error token.  */
      if (yyssp == yyss)
        YYABORT;


      yydestruct ("Error: popping",
                  yystos[yystate], yyvsp, domain);
      YYPOPSTACK (1);
      yystate = *yyssp;
      YY_STACK_PRINT (yyss, yyssp);
    }

  YY_IGNORE_MAYBE_UNINITIALIZED_BEGIN
  *++yyvsp = yylval;
  YY_IGNORE_MAYBE_UNINITIALIZED_END


  /* Shift the error token.  */
  YY_SYMBOL_PRINT ("Shifting", yystos[yyn], yyvsp, yylsp);

  yystate = yyn;
  goto yynewstate;


/*-------------------------------------.
| yyacceptlab -- YYACCEPT comes here.  |
`-------------------------------------*/
yyacceptlab:
  yyresult = 0;
  goto yyreturn;

/*-----------------------------------.
| yyabortlab -- YYABORT comes here.  |
`-----------------------------------*/
yyabortlab:
  yyresult = 1;
  goto yyreturn;

#if !defined yyoverflow || YYERROR_VERBOSE
/*-------------------------------------------------.
| yyexhaustedlab -- memory exhaustion comes here.  |
`-------------------------------------------------*/
yyexhaustedlab:
  yyerror (domain, YY_("memory exhausted"));
  yyresult = 2;
  /* Fall through.  */
#endif

yyreturn:
  if (yychar != YYEMPTY)
    {
      /* Make sure we have latest lookahead translation.  See comments at
         user semantic actions for why this is necessary.  */
      yytoken = YYTRANSLATE (yychar);
      yydestruct ("Cleanup: discarding lookahead",
                  yytoken, &yylval, domain);
    }
  /* Do not reclaim the symbols of the rule whose action triggered
     this YYABORT or YYACCEPT.  */
  YYPOPSTACK (yylen);
  YY_STACK_PRINT (yyss, yyssp);
  while (yyssp != yyss)
    {
      yydestruct ("Cleanup: popping",
                  yystos[*yyssp], yyvsp, domain);
      YYPOPSTACK (1);
    }
#ifndef yyoverflow
  if (yyss != yyssa)
    YYSTACK_FREE (yyss);
#endif
#if YYERROR_VERBOSE
  if (yymsg != yymsgbuf)
    YYSTACK_FREE (yymsg);
#endif
  return yyresult;
}
#line 195 "parser.y" /* yacc.c:1906  */


void parser_read_pddl_file(PDDLDomain *domain, char *file, int pipefd)
{
  if (file != NULL) {
    // Parade to find where is bzcat...
    char bzcat[512];
    short usrBzcat = 0;
    if ((pddl_in = fopen("/bin/bzcat", "r")) == NULL) 
    {
        if((pddl_in = fopen("/usr/bin/bzcat", "r")) == NULL)
            error(no_file, "Command 'bzcat' not found");
        else
        usrBzcat = 1;
    }

    if (usrBzcat)
    {
        strcpy( bzcat, "/usr/bin/bzcat -qf %s 2>/dev/null" );
    } else {
        strcpy( bzcat, "/bin/bzcat -qf %s 2>/dev/null" );
    }

    fclose(pddl_in);
    if ((pddl_in = fopen(file, "r")) == NULL) error(no_file, "File '%s' cannot be opened", file);
    fclose(pddl_in);
    // char bzcat[] = "/bin/bzcat -qf %s 2>/dev/null";
    char cmd[strlen(bzcat) + strlen(file)];
    sprintf(cmd, bzcat, file);
    pddl_in = popen(cmd, "r");
  } else pddl_in = fdopen(pipefd, "r");
  parser_reset_lineno();
  pddl_parse(domain);
  fclose(pddl_in);
}

