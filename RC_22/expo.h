//
//  expo.h
//  DOG_LCD
//
//  Created by Ruedi Heimlicher on 22.11.2013.
//
//

#ifndef DOG_LCD_expo_h
#define DOG_LCD_expo_h



const uint16_t expoarray[4][513] = 
{{0,1,1,2,3,3,4,5,5,6,7,7,8,9,10,10,11,12,12,13,14,14,15,16,17,17,18,19,19,20,21,21,22,23,24,24,25,26,26,27,28,29,29,30,31,31,32,33,34,34,35,36,37,37,38,39,39,40,41,42,42,43,44,45,45,46,47,48,48,49,50,51,51,52,53,54,54,55,56,57,57,58,59,60,60,61,62,63,63,64,65,66,67,67,68,69,70,70,71,72,73,74,74,75,76,77,77,78,79,80,81,81,82,83,84,85,85,86,87,88,89,89,90,91,92,93,93,94,95,96,97,97,98,99,100,101,102,102,103,104,105,106,107,107,108,109,110,111,112,112,113,114,115,116,117,117,118,119,120,121,122,123,123,124,125,126,127,128,129,129,130,131,132,133,134,135,135,136,137,138,139,140,141,142,142,143,144,145,146,147,148,149,150,150,151,152,153,154,155,156,157,158,159,159,160,161,162,163,164,165,166,167,168,169,169,170,171,172,173,174,175,176,177,178,179,180,181,182,183,183,184,185,186,187,188,189,190,191,192,193,194,195,196,197,198,199,200,201,202,203,204,204,205,206,207,208,209,210,211,212,213,214,215,216,217,218,219,220,221,222,223,224,225,226,227,228,229,230,231,232,233,234,235,236,237,238,239,240,241,242,243,244,245,246,248,249,250,251,252,253,254,255,256,257,258,259,260,261,262,263,264,265,266,267,268,270,271,272,273,274,275,276,277,278,279,280,281,282,284,285,286,287,288,289,290,291,292,293,294,296,297,298,299,300,301,302,303,304,306,307,308,309,310,311,312,313,315,316,317,318,319,320,321,322,324,325,326,327,328,329,330,332,333,334,335,336,337,339,340,341,342,343,344,346,347,348,349,350,351,353,354,355,356,357,359,360,361,362,363,365,366,367,368,369,371,372,373,374,375,377,378,379,380,382,383,384,385,386,388,389,390,391,393,394,395,396,398,399,400,401,403,404,405,406,408,409,410,412,413,414,415,417,418,419,420,422,423,424,426,427,428,429,431,432,433,435,436,437,439,440,441,442,444,445,446,448,449,450,452,453,454,456,457,458,460,461,462,464,465,466,468,469,470,472,473,475,476,477,479,480,481,483,484,486,487,488,490,491,492,494,495,497,498,499,501,502,504,505,506,508,509,511,512},
{0,0,1,1,2,2,3,3,4,4,4,5,5,6,6,7,7,8,8,9,9,9,10,10,11,11,12,12,13,13,14,14,15,15,16,16,17,17,18,18,19,19,20,20,21,21,22,22,23,23,24,24,25,25,26,26,27,27,28,28,29,29,30,30,31,31,32,32,33,33,34,34,35,36,36,37,37,38,38,39,39,40,40,41,42,42,43,43,44,44,45,46,46,47,47,48,48,49,50,50,51,51,52,52,53,54,54,55,55,56,57,57,58,58,59,60,60,61,62,62,63,63,64,65,65,66,66,67,68,68,69,70,70,71,72,72,73,74,74,75,75,76,77,77,78,79,79,80,81,81,82,83,83,84,85,86,86,87,88,88,89,90,90,91,92,92,93,94,95,95,96,97,97,98,99,100,100,101,102,103,103,104,105,105,106,107,108,108,109,110,111,111,112,113,114,115,115,116,117,118,118,119,120,121,122,122,123,124,125,125,126,127,128,129,129,130,131,132,133,134,134,135,136,137,138,139,139,140,141,142,143,144,144,145,146,147,148,149,150,151,151,152,153,154,155,156,157,158,158,159,160,161,162,163,164,165,166,167,168,168,169,170,171,172,173,174,175,176,177,178,179,180,181,182,183,184,185,185,186,187,188,189,190,191,192,193,194,195,196,197,198,199,200,201,202,203,204,205,207,208,209,210,211,212,213,214,215,216,217,218,219,220,221,222,223,225,226,227,228,229,230,231,232,233,234,236,237,238,239,240,241,242,243,245,246,247,248,249,250,251,253,254,255,256,257,258,260,261,262,263,264,266,267,268,269,270,272,273,274,275,277,278,279,280,282,283,284,285,287,288,289,290,292,293,294,296,297,298,299,301,302,303,305,306,307,309,310,311,313,314,315,317,318,319,321,322,323,325,326,328,329,330,332,333,335,336,337,339,340,342,343,344,346,347,349,350,352,353,355,356,357,359,360,362,363,365,366,368,369,371,372,374,375,377,378,380,381,383,385,386,388,389,391,392,394,395,397,399,400,402,403,405,407,408,410,411,413,415,416,418,420,421,423,425,426,428,430,431,433,435,436,438,440,441,443,445,447,448,450,452,454,455,457,459,461,462,464,466,468,469,471,473,475,477,478,480,482,484,486,488,489,491,493,495,497,499,501,503,504,506,508,510,512},
{0,0,1,1,1,1,2,2,2,2,3,3,3,4,4,4,5,5,5,5,6,6,6,7,7,7,7,8,8,8,9,9,9,10,10,10,11,11,11,12,12,12,13,13,13,14,14,14,15,15,15,16,16,16,17,17,17,18,18,18,19,19,19,20,20,20,21,21,21,22,22,23,23,23,24,24,24,25,25,26,26,26,27,27,28,28,28,29,29,30,30,30,31,31,32,32,32,33,33,34,34,34,35,35,36,36,37,37,37,38,38,39,39,40,40,41,41,41,42,42,43,43,44,44,45,45,46,46,46,47,47,48,48,49,49,50,50,51,51,52,52,53,53,54,54,55,55,56,56,57,57,58,58,59,59,60,61,61,62,62,63,63,64,64,65,65,66,67,67,68,68,69,69,70,71,71,72,72,73,73,74,75,75,76,76,77,78,78,79,79,80,81,81,82,83,83,84,84,85,86,86,87,88,88,89,90,90,91,92,92,93,94,94,95,96,96,97,98,98,99,100,101,101,102,103,103,104,105,106,106,107,108,109,109,110,111,112,112,113,114,115,115,116,117,118,118,119,120,121,122,122,123,124,125,126,126,127,128,129,130,131,131,132,133,134,135,136,137,137,138,139,140,141,142,143,144,144,145,146,147,148,149,150,151,152,153,154,155,156,156,157,158,159,160,161,162,163,164,165,166,167,168,169,170,171,172,173,174,175,176,177,178,179,180,181,183,184,185,186,187,188,189,190,191,192,193,194,196,197,198,199,200,201,202,204,205,206,207,208,209,211,212,213,214,215,217,218,219,220,221,223,224,225,226,228,229,230,231,233,234,235,237,238,239,240,242,243,244,246,247,248,250,251,252,254,255,257,258,259,261,262,264,265,266,268,269,271,272,274,275,276,278,279,281,282,284,285,287,288,290,291,293,295,296,298,299,301,302,304,306,307,309,310,312,314,315,317,318,320,322,323,325,327,328,330,332,334,335,337,339,340,342,344,346,348,349,351,353,355,356,358,360,362,364,366,368,369,371,373,375,377,379,381,383,385,387,388,390,392,394,396,398,400,402,404,406,408,410,412,415,417,419,421,423,425,427,429,431,433,436,438,440,442,444,446,449,451,453,455,458,460,462,464,467,469,471,474,476,478,481,483,485,488,490,492,495,497,500,502,505,507,510,512},
   {0,0,0,0,1,1,1,1,1,2,2,2,2,2,2,3,3,3,3,3,3,4,4,4,4,4,5,5,5,5,5,6,6,6,6,6,7,7,7,7,7,8,8,8,8,8,9,9,9,9,9,10,10,10,10,11,11,11,11,11,12,12,12,12,13,13,13,13,14,14,14,14,15,15,15,15,16,16,16,16,17,17,17,17,18,18,18,18,19,19,19,20,20,20,20,21,21,21,21,22,22,22,23,23,23,24,24,24,24,25,25,25,26,26,26,27,27,27,28,28,28,29,29,29,30,30,30,31,31,31,32,32,32,33,33,33,34,34,34,35,35,35,36,36,37,37,37,38,38,38,39,39,40,40,40,41,41,42,42,42,43,43,44,44,44,45,45,46,46,47,47,47,48,48,49,49,50,50,51,51,52,52,52,53,53,54,54,55,55,56,56,57,57,58,58,59,59,60,60,61,61,62,62,63,63,64,64,65,65,66,67,67,68,68,69,69,70,70,71,72,72,73,73,74,75,75,76,76,77,78,78,79,79,80,81,81,82,83,83,84,84,85,86,86,87,88,88,89,90,90,91,92,93,93,94,95,95,96,97,98,98,99,100,100,101,102,103,103,104,105,106,107,107,108,109,110,110,111,112,113,114,115,115,116,117,118,119,120,120,121,122,123,124,125,126,127,127,128,129,130,131,132,133,134,135,136,137,138,139,140,140,141,142,143,144,145,146,147,148,149,150,152,153,154,155,156,157,158,159,160,161,162,163,164,166,167,168,169,170,171,172,173,175,176,177,178,179,181,182,183,184,185,187,188,189,190,192,193,194,196,197,198,199,201,202,203,205,206,207,209,210,212,213,214,216,217,219,220,221,223,224,226,227,229,230,232,233,235,236,238,239,241,242,244,246,247,249,250,252,254,255,257,258,260,262,263,265,267,269,270,272,274,275,277,279,281,283,284,286,288,290,292,293,295,297,299,301,303,305,307,309,311,313,314,316,318,320,322,324,327,329,331,333,335,337,339,341,343,345,348,350,352,354,356,359,361,363,365,368,370,372,374,377,379,381,384,386,389,391,393,396,398,401,403,406,408,411,413,416,418,421,424,426,429,432,434,437,440,442,445,448,450,453,456,459,462,464,467,470,473,476,479,482,485,488,491,494,497,500,503,506,509,512}};



#endif


