grammar AlicaConf;

@header { package de.unikassel.vs.alica.generator.cpp.parser; }

options{
language = Java;
}

all : (headStart | COMMENT | headEnd | keyValue | (keyValue COMMENT) | '\n')+;

all_text: (headStart  all  headEnd)* ;

headStart: '['TEXT']'+;
headEnd: '[!'TEXT']'+;

COMMENT : ('#') ~'\n'* -> channel(HIDDEN);
keyValue: ((TEXT | KeyWithNumbers ) '=' (TEXT | INT| DOUBLE | PATH | BOOL))+;

KeyWithNumbers: TEXT INT TEXT;
PATH: TEXT ('/' TEXT)+ | ('/' TEXT)+;
BOOL : 'true' | 'false' ;
INT: [0-9]+;
TEXT : [a-zA-Z$]+;
WHITESPACE : ( '\t' | '\r' | ' ' | ',' | ':' | '(' | ')' | '_'  ) -> skip;
PT  : '.';
DOUBLE : INT+ PT INT+
    | PT INT+
    | INT+
    ;