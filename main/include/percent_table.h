// percentage factors in Q15 format
int16_t percent_table[] = {
0,
328,
655,
983,
1311,
1638,
1966,
2294,
2621,
2949,
3277,
3604,
3932,
4260,
4588,
4915,
5243,
5571,
5898,
6226,
6554,
6881,
7209,
7537,
7864,
8192,
8520,
8847,
9175,
9503,
9830,
10158,
10486,
10813,
11141,
11469,
11796,
12124,
12452,
12780,
13107,
13435,
13763,
14090,
14418,
14746,
15073,
15401,
15729,
16056,
16384,
16712,
17039,
17367,
17695,
18022,
18350,
18678,
19005,
19333,
19661,
19988,
20316,
20644,
20972,
21299,
21627,
21955,
22282,
22610,
22938,
23265,
23593,
23921,
24248,
24576,
24904,
25231,
25559,
25887,
26214,
26542,
26870,
27197,
27525,
27853,
28180,
28508,
28836,
29164,
29491,
29819,
30147,
30474,
30802,
31130,
31457,
31785,
32113,
32440,
32767
};
// sqrt of percentage factors in Q15 format
int16_t percent_sqrt_table[] = {
    0,
3277,
4634,
5676,
6554,
7327,
8026,
8670,
9268,
9830,
10362,
10868,
11351,
11815,
12261,
12691,
13107,
13511,
13902,
14283,
14654,
15016,
15370,
15715,
16053,
16384,
16708,
17027,
17339,
17646,
17948,
18244,
18536,
18824,
19107,
19386,
19661,
19932,
20200,
20464,
20724,
20982,
21236,
21487,
21736,
21981,
22224,
22465,
22702,
22938,
23170,
23401,
23629,
23855,
24079,
24301,
24521,
24739,
24955,
25170,
25382,
25593,
25802,
26009,
26214,
26418,
26621,
26822,
27021,
27219,
27416,
27611,
27805,
27997,
28188,
28378,
28566,
28754,
28940,
29125,
29309,
29491,
29673,
29853,
30032,
30211,
30388,
30564,
30739,
30913,
31086,
31259,
31430,
31600,
31770,
31938,
32106,
32273,
32439,
32604,
32767
};