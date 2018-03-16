//----------------------------------------------------------------------------
//
//  $Workfile: Red11Path60InPerSec.java
//
//  $Revision: X$
//
//  Project:    Paths
//
//                            Copyright (c) 2018
//                               Cedarcrest High School
//                            All Rights Reserved

//
//  Modification History:
//  $Log:
//  $
//
//----------------------------------------------------------------------------
//
//
//----------------------------------------------------------------------------
//    Parameters Used
//----------------------------------------------------------------------------
//   Time Slice= 0.020000
//   Max Vel   = 60.000000
//   Max Accel = 100.000000
//   Max Jerk  =  1000.000000
package org.usfirst.frc4089.Stealth2018.MPPaths;

public class Red11Path60InPerSec extends Path {
    public Red11Path60InPerSec() {
       kSpeed = 6.000000;
       kNumPoints = 162;
       kPoints = new double[][]{
  {0.400000, 0.400000, 0.002675, 16.003018, 287.250000, 16.003671, 273.250000},
  {0.721853, 0.878147, 0.015467, 16.017455, 287.250002, 16.021234, 273.250003},
  {1.624280, 1.975720, 0.044233, 16.049940, 287.250019, 16.060748, 273.250023},
  {2.887984, 3.512015, 0.095311, 16.107700, 287.250090, 16.130989, 273.250109},
  {4.513441, 5.486558, 0.174961, 16.197968, 287.250303, 16.240720, 273.250368},
  {6.320806, 7.679191, 0.286147, 16.324384, 287.250811, 16.394302, 273.250986},
  {8.130183, 9.869813, 0.428538, 16.486984, 287.251826, 16.591695, 273.252217},
  {9.942177, 12.057815, 0.601705, 16.685820, 287.253614, 16.832841, 273.254386},
  {11.757417, 14.242569, 0.805118, 16.920950, 287.256501, 17.117671, 273.257883},
  {13.576560, 16.423419, 1.038136, 17.192446, 287.260869, 17.446097, 273.263167},
  {15.400292, 18.599679, 1.300010, 17.500388, 287.267155, 17.818013, 273.270758},
  {17.229330, 20.770630, 1.589870, 17.844865, 287.275845, 18.233293, 273.281235},
  {19.064426, 22.935521, 1.906723, 18.225976, 287.287480, 18.691790, 273.295232},
  {20.906366, 25.093565, 2.249451, 18.643828, 287.302645, 19.193331, 273.313434},
  {22.755971, 27.243943, 2.616797, 19.098537, 287.321971, 19.737719, 273.336569},
  {24.614091, 29.385805, 3.007368, 19.590226, 287.346127, 20.324727, 273.365408},
  {26.481606, 31.518269, 3.419625, 20.119025, 287.375823, 20.954101, 273.400751},
  {28.359420, 33.640433, 3.851883, 20.685071, 287.411798, 21.625555, 273.443423},
  {30.248457, 35.751373, 4.302304, 21.288509, 287.454820, 22.338772, 273.494271},
  {32.149651, 37.850155, 4.768898, 21.929487, 287.505679, 23.093404, 273.554145},
  {34.063945, 39.935838, 5.249520, 22.608163, 287.565179, 23.889069, 273.623900},
  {35.992276, 42.007484, 5.741873, 23.324698, 287.634136, 24.725355, 273.704378},
  {37.935571, 44.064167, 6.243507, 24.079261, 287.713367, 25.601820, 273.796405},
  {39.894739, 46.104979, 6.751824, 24.872028, 287.803681, 26.517994, 273.900775},
  {41.870661, 48.129039, 7.264081, 25.703182, 287.905878, 27.473380, 274.018243},
  {43.864185, 50.135500, 7.777397, 26.572915, 288.020731, 28.467461, 274.149513},
  {45.876119, 52.123556, 8.288758, 27.481429, 288.148985, 29.499698, 274.295228},
  {47.907226, 54.092442, 8.795027, 28.428939, 288.291339, 30.569540, 274.455956},
  {49.958225, 56.041442, 9.292946, 29.415675, 288.448445, 31.676428, 274.632186},
  {52.029785, 57.969885, 9.779152, 30.441883, 288.620887, 32.819796, 274.824310},
  {53.932274, 59.667408, 10.248580, 31.504087, 288.808503, 33.994955, 275.031872},
  {55.472329, 60.927376, 10.695083, 32.595044, 289.010206, 35.193196, 275.253403},
  {56.644337, 61.755400, 11.113430, 33.707465, 289.224573, 36.405992, 275.487108},
  {57.442387, 62.157387, 11.499359, 34.834004, 289.449874, 37.625001, 275.730897},
  {57.860398, 62.139415, 11.849602, 35.967260, 289.684099, 38.842067, 275.982440},
  {58.086244, 61.913607, 12.162876, 37.103558, 289.925822, 40.053239, 276.240086},
  {58.311592, 61.688292, 12.439263, 38.243002, 290.174349, 41.258665, 276.503001},
  {58.536412, 61.463500, 12.678848, 39.385703, 290.428982, 42.458508, 276.770363},
  {58.760708, 61.239229, 12.881718, 40.531788, 290.689017, 43.652935, 277.041363},
  {58.984512, 61.015446, 13.047953, 41.681391, 290.953746, 44.842121, 277.315206},
  {59.207878, 60.792096, 13.177623, 42.834658, 291.222456, 46.026246, 277.591104},
  {59.430883, 60.569103, 13.270787, 43.991741, 291.494429, 47.205491, 277.868285},
  {59.653617, 60.346378, 13.327490, 45.152804, 291.768940, 48.380037, 278.145982},
  {59.876179, 60.123820, 13.347760, 46.318012, 292.045257, 49.550065, 278.423442},
  {60.098679, 59.901320, 13.331606, 47.487541, 292.322642, 50.715753, 278.699917},
  {60.321226, 59.678770, 13.279020, 48.661568, 292.600351, 51.877275, 278.974668},
  {60.543927, 59.456061, 13.189977, 49.840272, 292.877628, 53.034800, 279.246964},
  {60.766884, 59.233092, 13.064435, 51.023837, 293.153710, 54.188490, 279.516079},
  {60.990190, 59.009770, 12.902335, 52.212442, 293.427823, 55.338500, 279.781294},
  {61.213921, 58.786019, 12.703609, 53.406268, 293.699185, 56.484975, 280.041895},
  {61.438135, 58.561781, 12.468176, 54.605490, 293.966998, 57.628053, 280.297173},
  {61.662867, 58.337020, 12.195951, 55.810278, 294.230457, 58.767858, 280.546425},
  {61.888123, 58.111731, 11.886849, 57.020793, 294.488740, 59.904507, 280.788952},
  {62.113878, 57.885939, 11.540787, 58.237185, 294.741014, 61.038102, 281.024059},
  {62.340068, 57.659708, 11.157693, 59.459594, 294.986434, 62.168733, 281.251058},
  {62.566587, 57.433144, 10.737514, 60.688141, 295.224140, 63.296480, 281.469266},
  {62.793281, 57.206400, 10.280221, 61.922929, 295.453258, 64.421405, 281.678004},
  {63.019946, 56.979682, 9.785817, 63.164042, 295.672903, 65.543560, 281.876603},
  {63.246320, 56.753250, 9.254350, 64.411537, 295.882177, 66.662982, 282.064399},
  {63.472080, 56.527427, 8.685920, 65.665443, 296.080172, 67.779694, 282.240737},
  {63.696842, 56.302600, 8.080690, 66.925758, 296.265968, 68.893704, 282.404973},
  {63.920150, 56.079222, 7.438898, 68.192446, 296.438642, 70.005010, 282.556473},
  {64.141484, 55.857816, 6.760867, 69.465432, 296.597261, 71.113592, 282.694615},
  {64.360250, 55.638974, 6.047016, 70.744598, 296.740894, 72.219421, 282.818793},
  {64.575787, 55.423358, 5.297874, 72.029783, 296.868610, 73.322454, 282.928416},
  {64.787366, 55.211698, 4.514089, 73.320778, 296.979483, 74.422638, 283.022911},
  {64.994195, 55.004786, 3.696438, 74.617323, 297.072599, 75.519907, 283.101724},
  {65.195426, 54.803472, 2.845837, 75.919104, 297.147058, 76.614187, 283.164323},
  {65.390162, 54.608652, 1.963350, 77.225753, 297.201982, 77.705396, 283.210201},
  {65.577468, 54.421261, 1.050192, 78.536848, 297.236522, 78.793444, 283.238874},
  {65.756387, 54.242260, 0.107737, 79.851908, 297.249861, 79.878233, 283.249885},
  {50.312853, 69.642118, 1.689914, 80.858081, 297.262876, 81.270945, 283.268965},
  {49.419753, 70.575679, 3.421616, 81.845478, 297.307273, 82.681040, 283.332230},
  {50.553634, 69.442725, 4.967756, 82.853818, 297.381552, 84.066150, 283.434142},
  {51.686684, 68.310495, 6.328466, 83.882502, 297.483612, 85.425695, 283.568923},
  {52.808370, 67.189519, 7.505599, 84.930944, 297.611124, 86.759667, 283.731074},
  {53.912392, 66.086095, 8.502043, 85.998644, 297.761567, 88.068470, 283.915419},
  {54.995973, 65.003005, 9.321137, 87.085239, 297.932259, 89.352789, 284.117115},
  {56.059114, 63.940251, 9.966220, 88.190527, 298.120377, 90.613472, 284.331637},
  {57.103929, 62.895728, 10.440287, 89.314491, 298.322977, 91.851441, 284.554756},
  {58.134060, 61.865797, 10.745734, 90.457304, 298.537002, 93.067616, 284.782502},
  {59.154189, 60.845781, 10.884192, 91.619320, 298.759278, 94.262863, 285.011126},
  {60.169627, 59.830371, 10.856424, 92.801065, 298.986507, 95.437944, 285.237075},
  {61.185962, 58.813980, 10.662275, 94.003216, 299.215242, 96.593491, 285.456954},
  {62.208724, 57.791076, 10.300684, 95.226577, 299.441868, 97.729972, 285.667507},
  {63.243055, 56.756515, 9.769752, 96.472035, 299.662567, 98.847684, 285.865601},
  {64.293333, 55.705914, 9.066858, 97.740518, 299.873287, 99.946735, 286.048215},
  {65.362741, 54.636084, 8.188860, 99.032933, 300.069701, 101.027044, 286.212446},
  {66.452752, 53.545547, 7.132376, 100.350085, 300.247182, 102.088355, 286.355515},
  {67.562511, 52.435153, 5.894161, 101.692577, 300.400777, 103.130253, 286.474791},
  {68.688145, 51.308772, 4.471603, 103.060694, 300.525198, 104.152204, 286.567812},
  {69.822017, 50.174043, 2.863342, 104.454254, 300.614846, 105.153609, 286.632325},
  {70.952041, 49.043060, 1.069994, 105.872448, 300.663865, 106.133881, 286.666307},
  {72.061180, 47.932878, 359.094968, 107.313669, 300.666252, 107.092537, 286.667999},
  {73.127349, 46.865611, 356.945289, 108.775353, 300.616020, 108.029300, 286.635913},
  {74.123943, 45.867906, 354.632344, 110.253850, 300.507429, 108.944202, 286.568820},
  {75.021198, 44.969583, 352.172397, 111.744365, 300.335274, 109.837665, 286.465721},
  {75.788467, 44.201346, 349.586736, 113.241002, 300.095205, 110.710546, 286.325790},
  {76.397279, 43.591733, 346.901320, 114.736928, 299.784041, 111.564124, 286.148305},
  {76.824742, 43.163690, 344.145860, 116.224664, 299.400038, 112.400013, 285.932594},
  {77.056655, 42.931456, 341.352398, 117.696481, 298.943042, 113.220028, 285.677999},
  {77.089612, 42.898452, 338.553535, 119.144852, 298.414511, 114.026008, 285.383877},
  {76.931554, 43.056731, 335.780572, 120.562889, 297.817387, 114.819637, 285.049652},
  {76.600633, 43.388106, 333.061835, 121.944702, 297.155831, 115.602301, 284.674888},
  {76.122718, 43.866661, 330.421407, 123.285639, 296.434891, 116.375001, 284.259379},
  {62.771737, 57.181091, 329.963803, 124.373222, 295.807771, 117.365564, 283.687840},
  {60.814873, 59.185099, 329.830404, 125.425559, 295.197880, 118.389701, 283.094298},
  {61.422387, 58.577531, 329.597550, 126.486435, 294.578504, 119.401445, 282.503616},
  {62.044195, 57.955634, 329.262896, 127.554940, 293.947562, 120.399545, 281.914261},
  {62.686619, 57.313087, 328.823065, 128.630179, 293.302809, 121.382622, 281.324790},
  {63.356232, 56.643308, 328.273603, 129.711241, 292.641817, 122.349150, 280.733852},
  {64.059917, 55.939410, 327.608927, 130.797172, 291.961947, 123.297438, 280.140187},
  {64.804880, 55.194177, 326.822274, 131.886938, 291.260321, 124.225607, 279.542641},
  {65.598630, 54.400091, 325.905651, 132.979381, 290.533795, 125.131578, 278.940176},
  {66.448872, 53.549430, 324.849802, 134.073160, 289.778925, 126.013054, 278.331886},
  {67.363304, 52.634482, 323.644209, 135.166683, 288.991948, 126.867516, 277.717028},
  {68.349244, 51.647910, 322.277154, 136.258017, 288.168761, 127.692223, 277.095046},
  {69.413020, 50.583362, 320.735879, 137.344776, 287.304919, 128.484230, 276.465605},
  {70.559062, 49.436385, 319.006899, 138.423984, 286.395665, 129.240429, 275.828625},
  {71.788574, 48.205751, 317.076527, 139.491912, 285.435993, 129.957619, 275.184298},
  {73.097739, 46.895255, 314.931699, 140.543894, 284.420787, 130.632605, 274.533100},
  {74.475457, 45.515984, 312.561165, 141.574133, 283.345033, 131.262353, 273.875756},
  {75.900752, 44.088920, 309.957107, 142.575521, 282.204160, 131.844165, 273.213165},
  {77.340272, 42.647443, 307.117169, 143.539535, 280.994497, 132.375891, 272.546240},
  {78.746659, 41.238980, 304.046756, 144.456256, 279.713857, 132.856122, 271.875688},
  {80.058898, 39.924657, 300.761274, 145.314605, 278.362189, 133.284324, 271.201718},
  {80.958109, 38.659650, 297.298571, 146.100588, 276.946591, 133.659786, 270.525808},
  {81.086589, 37.402081, 293.722365, 146.799220, 275.483059, 133.982141, 269.850787},
  {80.349122, 36.211409, 290.109045, 147.399122, 273.992250, 134.252562, 269.178939},
  {78.718500, 35.114881, 286.539463, 147.894117, 272.497720, 134.473383, 268.512261},
  {76.217869, 34.089215, 283.090665, 148.283843, 271.024024, 134.647663, 267.853128},
  {73.163335, 33.181916, 279.817696, 148.574318, 269.589879, 134.779344, 267.202685},
  {69.911808, 32.435679, 276.749867, 148.775631, 268.206210, 134.872668, 266.560719},
  {66.564836, 31.784760, 273.902786, 148.899057, 266.880648, 134.931523, 265.927754},
  {63.187173, 31.164314, 271.281451, 148.956054, 265.618190, 134.959555, 265.305099},
  {59.828292, 30.524827, 268.882754, 148.957620, 264.421625, 134.960281, 264.694603},
  {56.522479, 29.832007, 266.697977, 148.913900, 263.292021, 134.937143, 264.098411},
  {53.290936, 29.064671, 264.714931, 148.833988, 262.229203, 134.893506, 263.518758},
  {50.144657, 28.211855, 262.919633, 148.725876, 261.232154, 134.832637, 262.957814},
  {47.087268, 27.269966, 261.297511, 148.596494, 260.299339, 134.757671, 262.417591},
  {44.117452, 26.240354, 259.834211, 148.451795, 259.428935, 134.671579, 261.899894},
  {41.230814, 25.127440, 258.516103, 148.296870, 258.619003, 134.577141, 261.406298},
  {38.421236, 23.937369, 257.330561, 148.136055, 257.867594, 134.476932, 260.938156},
  {35.681799, 22.677080, 256.266095, 147.973040, 257.172826, 134.373317, 260.496609},
  {33.005381, 21.353709, 255.312381, 147.810964, 256.532925, 134.268448, 260.082610},
  {30.385029, 19.974224, 254.460238, 147.652494, 255.946251, 134.164267, 259.696949},
  {27.814158, 18.545221, 253.701559, 147.499901, 255.411305, 134.062520, 259.340274},
  {25.286658, 17.072816, 253.029243, 147.355114, 254.926741, 133.964760, 259.013111},
  {22.796922, 15.562624, 252.437105, 147.219773, 254.491353, 133.872365, 258.715888},
  {20.339842, 14.019759, 251.919798, 147.095266, 254.104078, 133.786544, 258.448950},
  {17.910772, 12.448868, 251.472734, 146.982763, 253.763988, 133.708347, 258.212571},
  {15.505486, 10.854183, 251.092019, 146.883245, 253.470280, 133.638682, 258.006969},
  {13.120125, 9.239564, 250.774390, 146.797524, 253.222274, 133.578315, 257.832316},
  {10.751145, 7.608558, 250.517165, 146.726263, 253.019403, 133.527883, 257.688745},
  {8.395261, 5.964450, 250.318201, 146.669988, 252.861209, 133.487902, 257.576356},
  {6.049401, 4.310314, 250.175855, 146.629098, 252.747340, 133.458767, 257.495222},
  {3.922983, 2.800762, 250.084000, 146.602430, 252.673552, 133.439728, 257.442541},
  {2.246196, 1.605606, 250.031566, 146.587108, 252.631322, 133.428775, 257.412355},
  {1.037652, 0.742207, 250.007384, 146.580017, 252.611818, 133.423703, 257.398404},
  {0.296080, 0.211835, 250.000488, 146.577992, 252.606253, 133.422254, 257.394423},
  {0.020969, 0.015003, 250.000000, 146.577848, 252.605859, 133.422152, 257.394141},
  {0.000000, 0.000000, 250.000000, 146.577848, 252.605859, 133.422152, 257.394141}};
}}
