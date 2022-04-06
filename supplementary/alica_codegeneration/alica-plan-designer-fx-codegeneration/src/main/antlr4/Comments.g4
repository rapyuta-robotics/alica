grammar Comments;

@header { package de.unikassel.vs.alica.generator.cpp.parser; }

options {
    language = Java;
}

all_text : ((protected_region+ code*) | (protected_region* code+))+;

code : ((TEXT+ NUMBER* SIGNS* WHITESPACE*) | (TEXT* NUMBER+ SIGNS* WHITESPACE*) | (TEXT* NUMBER* SIGNS+ WHITESPACE*) | (TEXT* NUMBER* SIGNS* WHITESPACE+))+;

protected_region : protected_region_header content=protected_region_content protected_region_end;

protected_region_header : '/*PROTECTED REGION ID(' id=regionid ') ENABLED START*/';

protected_region_content : code;

protected_region_end : '/*PROTECTED REGION END*/';

regionid : ((TEXT+ NUMBER*) | (TEXT* NUMBER+))+;

SIGNS : '*' | '+' | '-' | '/' | '\\' | '#' | '~' | '|' | '\'' | '"' | '[' | ']' | '=' | ';' | ',' | '&' | '$' | '.' | '(' | ')' | '{' | '}' | '!' | '?' | '<' | '>' | ':' | '@' | '_' | '%' | 'ä' | 'ü' | 'ß' | 'ö';
TEXT : [a-zA-Z$];
NUMBER : '0'..'9'+;
WHITESPACE : ( '\t' | '\r' | '\n' | '\u000C' | ' ' )+;