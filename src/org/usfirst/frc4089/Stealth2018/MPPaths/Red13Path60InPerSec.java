//----------------------------------------------------------------------------
//
//  $Workfile: Red13Path60InPerSec.java
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

public class Red13Path60InPerSec extends Path {
    public Red13Path60InPerSec() {
       kSpeed = 6.000000;
       kNumPoints = 234;
       kPoints = new double[][]{
  {0.400000, 0.400000, 0.002369, 16.003059, 287.250000, 16.003637, 273.250000},
  {0.730853, 0.869147, 0.013689, 16.017676, 287.250002, 16.021020, 273.250003},
  {1.644522, 1.955478, 0.039141, 16.050566, 287.250017, 16.060130, 273.250021},
  {2.923950, 3.476050, 0.084331, 16.109045, 287.250080, 16.129651, 273.250095},
  {4.569584, 5.430415, 0.154791, 16.200437, 287.250271, 16.238259, 273.250322},
  {6.399293, 7.600705, 0.253128, 16.328422, 287.250727, 16.390272, 273.250863},
  {8.230893, 9.769103, 0.379032, 16.493037, 287.251635, 16.585651, 273.251941},
  {10.064946, 11.935047, 0.532102, 16.694329, 287.253236, 16.824345, 273.253840},
  {11.902032, 14.097958, 0.711841, 16.932356, 287.255820, 17.106287, 273.256901},
  {13.742746, 16.257238, 0.917655, 17.207183, 287.259729, 17.431399, 273.261525},
  {15.587706, 18.412271, 1.148849, 17.518887, 287.265352, 17.799585, 273.268166},
  {17.437552, 20.562417, 1.404623, 17.867551, 287.273124, 18.210731, 273.277331},
  {19.292946, 22.707012, 1.684068, 18.253270, 287.283526, 18.664706, 273.289573},
  {21.154576, 24.845370, 1.986164, 18.676144, 287.297077, 19.161358, 273.305488},
  {23.023150, 26.976783, 2.309774, 19.136284, 287.314339, 19.700515, 273.325714},
  {24.899398, 29.100521, 2.653641, 19.633804, 287.335907, 20.281979, 273.350920},
  {26.784070, 31.215833, 3.016386, 20.168830, 287.362408, 20.905532, 273.381804},
  {28.677931, 33.321956, 3.396506, 20.741490, 287.394496, 21.570927, 273.419088},
  {30.581758, 35.418110, 3.792367, 21.351921, 287.432850, 22.277895, 273.463506},
  {32.496339, 37.503512, 4.202211, 22.000266, 287.478167, 23.026140, 273.515804},
  {34.422460, 39.577373, 4.624147, 22.686673, 287.531154, 23.815340, 273.576725},
  {36.360910, 41.638905, 5.056158, 23.411297, 287.592529, 24.645147, 273.647006},
  {38.312466, 43.687333, 5.496098, 24.174298, 287.663008, 25.515190, 273.727369},
  {40.277894, 45.721889, 5.941696, 24.975845, 287.743300, 26.425074, 273.818511},
  {42.257941, 47.741828, 6.390559, 25.816111, 287.834103, 27.374384, 273.921095},
  {44.253332, 49.746426, 6.840177, 26.695282, 287.936093, 28.362685, 274.035741},
  {46.264764, 51.734987, 7.287921, 27.613550, 288.049915, 29.389527, 274.163018},
  {48.292904, 53.706842, 7.731059, 28.571120, 288.176178, 30.454447, 274.303432},
  {50.338389, 55.661355, 8.166751, 29.568209, 288.315442, 31.556972, 274.457418},
  {52.401826, 57.597922, 8.592058, 30.605051, 288.468213, 32.696627, 274.625334},
  {54.292304, 59.307453, 9.002555, 31.678115, 288.634329, 33.868814, 274.806790},
  {55.815648, 60.584127, 9.392861, 32.780067, 288.812814, 35.064910, 275.000519},
  {56.966967, 61.432833, 9.758398, 33.903523, 289.002396, 36.276438, 275.204959},
  {57.741107, 61.858721, 10.095429, 35.041046, 289.201534, 37.495080, 275.418293},
  {58.132761, 61.867097, 10.401089, 36.185142, 289.408442, 38.712672, 275.638489},
  {58.331340, 61.668547, 10.674244, 37.332083, 289.621852, 39.925231, 275.864106},
  {58.529580, 61.470332, 10.914948, 38.481950, 289.841146, 41.132873, 276.094416},
  {58.727462, 61.272472, 11.123260, 39.634834, 290.065701, 42.335719, 276.328699},
  {58.924990, 61.074963, 11.299237, 40.790834, 290.294888, 43.533897, 276.566247},
  {59.122186, 60.877783, 11.442935, 41.950056, 290.528077, 44.727542, 276.806358},
  {59.319089, 60.680892, 11.554400, 43.112615, 290.764632, 45.916790, 277.048342},
  {59.515751, 60.484239, 11.633672, 44.278632, 291.003911, 47.101782, 277.291515},
  {59.712234, 60.287763, 11.680780, 45.448233, 291.245271, 48.282656, 277.535200},
  {59.908607, 60.091393, 11.695741, 46.621548, 291.488060, 49.459552, 277.778729},
  {60.104944, 59.895056, 11.678561, 47.798714, 291.731621, 50.632607, 278.021440},
  {60.301319, 59.698678, 11.629234, 48.979868, 291.975292, 51.801956, 278.262677},
  {60.497805, 59.502185, 11.547742, 50.165149, 292.218405, 52.967730, 278.501789},
  {60.694472, 59.305508, 11.434053, 51.354696, 292.460281, 54.130055, 278.738132},
  {60.891381, 59.108586, 11.288130, 52.548650, 292.700238, 55.289051, 278.971065},
  {61.088583, 58.911368, 11.109922, 53.747146, 292.937583, 56.444832, 279.199953},
  {61.286116, 58.713817, 10.899376, 54.950318, 293.171615, 57.597505, 279.424165},
  {61.484001, 58.515909, 10.656435, 56.158296, 293.401624, 58.747168, 279.643073},
  {61.682239, 58.317645, 10.381039, 57.371199, 293.626890, 59.893910, 279.856054},
  {61.880811, 58.119044, 10.073133, 58.589142, 293.846685, 61.037812, 280.062491},
  {62.079670, 57.920154, 9.732672, 59.812226, 294.060269, 62.178946, 280.261768},
  {62.278738, 57.721050, 9.359619, 61.040543, 294.266894, 63.317372, 280.453275},
  {62.477909, 57.521840, 8.953959, 62.274169, 294.465800, 64.453139, 280.636408},
  {62.677038, 57.322669, 8.515697, 63.513162, 294.656221, 65.586287, 280.810567},
  {62.875943, 57.123719, 8.044869, 64.757564, 294.837380, 66.716843, 280.975157},
  {63.074401, 56.925214, 7.541550, 66.007393, 295.008492, 67.844825, 281.129592},
  {63.272142, 56.727421, 7.005855, 67.262645, 295.168764, 68.970236, 281.273293},
  {63.468854, 56.530655, 6.437954, 68.523290, 295.317401, 70.093070, 281.405687},
  {63.664175, 56.335277, 5.838072, 69.789268, 295.453601, 71.213311, 281.526214},
  {63.857693, 56.141699, 5.206507, 71.060489, 295.576560, 72.330928, 281.634322},
  {64.048950, 55.950381, 4.543626, 72.336829, 295.685476, 73.445883, 281.729473},
  {64.237436, 55.761831, 3.849884, 73.618129, 295.779549, 74.558125, 281.811142},
  {64.422596, 55.576605, 3.125825, 74.904191, 295.857987, 75.667596, 281.878816},
  {64.603832, 55.395302, 2.372091, 76.194779, 295.920006, 76.774224, 281.932002},
  {64.780505, 55.218562, 1.589429, 77.489613, 295.964837, 77.877934, 281.970224},
  {64.951940, 55.047059, 0.778697, 78.788373, 295.991729, 78.978639, 281.993022},
  {64.640134, 55.258582, 0.010801, 80.081149, 296.000008, 80.083789, 282.000008},
  {59.066620, 60.933344, 0.163594, 81.262480, 296.001820, 81.302454, 282.001877},
  {59.116511, 60.883458, 0.308221, 82.444801, 296.006702, 82.520113, 282.006905},
  {59.166350, 60.833622, 0.444689, 83.628102, 296.014491, 83.736759, 282.014913},
  {59.216137, 60.783838, 0.573007, 84.812378, 296.025023, 84.952388, 282.025723},
  {59.265870, 60.734108, 0.693184, 85.997623, 296.038135, 86.166995, 282.039159},
  {59.315549, 60.684432, 0.805229, 87.183832, 296.053661, 87.380580, 282.055043},
  {59.365175, 60.634809, 0.909150, 88.371002, 296.071437, 88.593140, 282.073200},
  {59.414748, 60.585238, 1.004956, 89.559131, 296.091299, 89.804676, 282.093453},
  {59.464272, 60.535717, 1.092655, 90.748217, 296.113082, 91.015187, 282.115628},
  {59.513747, 60.486243, 1.172254, 91.938260, 296.136621, 92.224675, 282.139551},
  {59.563177, 60.436815, 1.243763, 93.129258, 296.161749, 93.433143, 282.165048},
  {59.612566, 60.387428, 1.307186, 94.321214, 296.188302, 94.640592, 282.191945},
  {59.661916, 60.338079, 1.362531, 95.514128, 296.216113, 95.847025, 282.220072},
  {59.711232, 60.288764, 1.409802, 96.708003, 296.245017, 97.052448, 282.249254},
  {59.760519, 60.239479, 1.449006, 97.902841, 296.274845, 98.256862, 282.279322},
  {59.809780, 60.190218, 1.480145, 99.098645, 296.305433, 99.460273, 282.310104},
  {59.859021, 60.140978, 1.503224, 100.295420, 296.336612, 100.662684, 282.341430},
  {59.908248, 60.091752, 1.518244, 101.493168, 296.368215, 101.864101, 282.373129},
  {59.957464, 60.042536, 1.525207, 102.691894, 296.400073, 103.064528, 282.405033},
  {60.006676, 59.993324, 1.524114, 103.891602, 296.432020, 104.263969, 282.436972},
  {60.055888, 59.944112, 1.514965, 105.092297, 296.463885, 105.462430, 282.468778},
  {60.105106, 59.894894, 1.497759, 106.293983, 296.495500, 106.659913, 282.500283},
  {60.154335, 59.845664, 1.472494, 107.496665, 296.526694, 107.856424, 282.531318},
  {60.203581, 59.796418, 1.439167, 108.700348, 296.557300, 109.051966, 282.561716},
  {60.252848, 59.747150, 1.397775, 109.905035, 296.587144, 110.246542, 282.591310},
  {60.302141, 59.697856, 1.348314, 111.110732, 296.616057, 111.440156, 282.619934},
  {60.351464, 59.648531, 1.290778, 112.317440, 296.643867, 112.632810, 282.647420},
  {60.400823, 59.599171, 1.225162, 113.525165, 296.670402, 113.824506, 282.673603},
  {60.450221, 59.549771, 1.151459, 114.733909, 296.695489, 115.015245, 282.698316},
  {60.499661, 59.500329, 1.069663, 115.943675, 296.718955, 116.205028, 282.721395},
  {60.549147, 59.450841, 0.979765, 117.154464, 296.740626, 117.393854, 282.742673},
  {60.598681, 59.401304, 0.881758, 118.366278, 296.760328, 118.581723, 282.761985},
  {60.648267, 59.351716, 0.775634, 119.579116, 296.777885, 119.768633, 282.779168},
  {60.697904, 59.302076, 0.661384, 120.792978, 296.793122, 120.954581, 282.794055},
  {60.747595, 59.252382, 0.538999, 122.007863, 296.805863, 122.139564, 282.806483},
  {60.797339, 59.202635, 0.408471, 123.223769, 296.815931, 123.323576, 282.816287},
  {60.847137, 59.152834, 0.269790, 124.440690, 296.823148, 124.506612, 282.823303},
  {60.896987, 59.102980, 0.122949, 125.658622, 296.827337, 125.688664, 282.827369},
  {60.946887, 59.053076, 359.967938, 126.877560, 296.828318, 126.869726, 282.828320},
  {60.996834, 59.003125, 359.804751, 128.097494, 296.825912, 128.049786, 282.825993},
  {61.046826, 58.953130, 359.633379, 129.318416, 296.819940, 129.228834, 282.820226},
  {61.096855, 58.903096, 359.453818, 130.540314, 296.810221, 130.406859, 282.810857},
  {61.146918, 58.853028, 359.266060, 131.763177, 296.796574, 131.583846, 282.797722},
  {61.197008, 58.802934, 359.070102, 132.986988, 296.778817, 132.759781, 282.780661},
  {61.247115, 58.752822, 358.865942, 134.211732, 296.756770, 133.934647, 282.759512},
  {61.297232, 58.702700, 358.653576, 135.437390, 296.730249, 135.108426, 282.734114},
  {61.347347, 58.652579, 358.433006, 136.663940, 296.699072, 136.281099, 282.704308},
  {61.397449, 58.602471, 358.204234, 137.891361, 296.663057, 137.452645, 282.669933},
  {61.447525, 58.552389, 357.967264, 139.119626, 296.622020, 138.623039, 282.630829},
  {61.497561, 58.502347, 357.722102, 140.348708, 296.575777, 139.792259, 282.586840},
  {61.547541, 58.452361, 357.468758, 141.578575, 296.524147, 140.960277, 282.537807},
  {61.597447, 58.402449, 357.207243, 142.809196, 296.466944, 142.127067, 282.483572},
  {61.647261, 58.352628, 356.937574, 144.040532, 296.403987, 143.292597, 282.423980},
  {61.696963, 58.302919, 356.659767, 145.272547, 296.335092, 144.456836, 282.358876},
  {61.746531, 58.253345, 356.373846, 146.505197, 296.260077, 145.619752, 282.288106},
  {61.795941, 58.203928, 356.079835, 147.738438, 296.178760, 146.781308, 282.211516},
  {61.845168, 58.154693, 355.777765, 148.972221, 296.090959, 147.941468, 282.128955},
  {61.894187, 58.105667, 355.467670, 150.206495, 295.996493, 149.100193, 282.040272},
  {61.942968, 58.056878, 355.149589, 151.441205, 295.895182, 150.257441, 281.945318},
  {61.991482, 58.008356, 354.823565, 152.676292, 295.786848, 151.413171, 281.843946},
  {62.039697, 57.960133, 354.489648, 153.911696, 295.671314, 152.567337, 281.736009},
  {62.087581, 57.912241, 354.147891, 155.147349, 295.548401, 153.719894, 281.621364},
  {62.135100, 57.864714, 353.798355, 156.383184, 295.417937, 154.870793, 281.499867},
  {62.182216, 57.817589, 353.441105, 157.619127, 295.279749, 156.019985, 281.371379},
  {62.228893, 57.770904, 353.076213, 158.855102, 295.133664, 157.167416, 281.235761},
  {62.275092, 57.724697, 352.703757, 160.091028, 294.979515, 158.313034, 281.092876},
  {62.320773, 57.679007, 352.323823, 161.326821, 294.817136, 159.456783, 280.942592},
  {62.365893, 57.633878, 351.936501, 162.562393, 294.646362, 160.598606, 280.784777},
  {62.410411, 57.589351, 351.541891, 163.797652, 294.467033, 161.738444, 280.619301},
  {62.454283, 57.545471, 351.140099, 165.032502, 294.278991, 162.876237, 280.446040},
  {62.497464, 57.502281, 350.731237, 166.266844, 294.082082, 164.011923, 280.264870},
  {62.539909, 57.459827, 350.315425, 167.500574, 293.876155, 165.145438, 280.075671},
  {62.581572, 57.418156, 349.892793, 168.733585, 293.661063, 166.276718, 279.878327},
  {62.622405, 57.377314, 349.463475, 169.965767, 293.436663, 167.405695, 279.672724},
  {62.662363, 57.337348, 349.027616, 171.197004, 293.202818, 168.532302, 279.458751},
  {62.701397, 57.298305, 348.585366, 172.427179, 292.959393, 169.656471, 279.236304},
  {62.739461, 57.260233, 348.136884, 173.656170, 292.706259, 170.778131, 279.005278},
  {62.776508, 57.223178, 347.682336, 174.883853, 292.443293, 171.897210, 278.765575},
  {62.812490, 57.187187, 347.221898, 176.110099, 292.170376, 173.013638, 278.517100},
  {62.847362, 57.152307, 346.755750, 177.334778, 291.887395, 174.127340, 278.259763},
  {62.881078, 57.118583, 346.284082, 178.557755, 291.594243, 175.238242, 277.993477},
  {62.913595, 57.086058, 345.807090, 179.778895, 291.290820, 176.346271, 277.718160},
  {62.944869, 57.054777, 345.324978, 180.998058, 290.977030, 177.451351, 277.433734},
  {63.929124, 55.955500, 344.672324, 182.233370, 290.647215, 178.532625, 277.145197},
  {66.320235, 53.678134, 343.637540, 183.509408, 290.285170, 179.565428, 276.852188},
  {66.751702, 53.246437, 342.532102, 184.786721, 289.896858, 180.584322, 276.542464},
  {67.223364, 52.774506, 341.349426, 186.064999, 289.480207, 181.587858, 276.215396},
  {67.738992, 52.258563, 340.082310, 187.343817, 289.032932, 182.574439, 275.870370},
  {68.302417, 51.694769, 338.722923, 188.622600, 288.552515, 183.542302, 275.506805},
  {68.917363, 51.079391, 337.262826, 189.900586, 288.036189, 184.489523, 275.124163},
  {69.587189, 50.409059, 335.693026, 191.176764, 287.480923, 185.414010, 274.721979},
  {70.314516, 49.681140, 334.004100, 192.449813, 286.883429, 186.313517, 274.299873},
  {71.100702, 48.894267, 332.186402, 193.718014, 286.240163, 187.185662, 273.857579},
  {71.945131, 48.049042, 330.230385, 194.979155, 285.547370, 188.027964, 273.394965},
  {72.844306, 47.148957, 328.127074, 196.230420, 284.801144, 188.837901, 272.912046},
  {73.790735, 46.201498, 325.868718, 197.468264, 283.997544, 189.612990, 272.408986},
  {74.771693, 45.219395, 323.449650, 198.688292, 283.132759, 190.350886, 271.886085},
  {75.768000, 44.221843, 320.867343, 199.885163, 282.203342, 191.049510, 271.343726},
  {76.753093, 43.235440, 318.123624, 201.052534, 281.206517, 191.707176, 270.782302},
  {77.692789, 42.294420, 315.225919, 202.183097, 280.140545, 192.322712, 270.202093},
  {78.546223, 41.439721, 312.188355, 203.268727, 279.005111, 192.895551, 269.603131},
  {79.268373, 40.716453, 309.032438, 204.300801, 277.801694, 193.425747, 268.985050},
  {79.814309, 40.169644, 305.787044, 205.270663, 276.533823, 193.913918, 268.346983},
  {80.144741, 39.838671, 302.487492, 206.170222, 275.207148, 194.361100, 267.687531},
  {80.231852, 39.751416, 299.173661, 206.992590, 273.829262, 194.768543, 267.004845},
  {80.063947, 39.919598, 295.887351, 207.732660, 272.409265, 195.137501, 266.296820},
  {79.647551, 40.336671, 292.669289, 208.387493, 270.957133, 195.469066, 265.561372},
  {79.006280, 40.978956, 289.556327, 208.956465, 269.483000, 195.764085, 264.796733},
  {78.176851, 41.809647, 286.579296, 209.441137, 267.996480, 196.023176, 264.001692},
  {77.203414, 42.784493, 283.761785, 209.844924, 266.506144, 196.246820, 263.175745},
  {76.131726, 43.857642, 281.119881, 210.172638, 265.019194, 196.435477, 262.319120},
  {75.004377, 44.986426, 278.662690, 210.430000, 263.541349, 196.589709, 261.432709},
  {73.857697, 46.134459, 276.393363, 210.623198, 262.076884, 196.710267, 260.517930},
  {72.720347, 47.273044, 274.310359, 210.758533, 260.628787, 196.798131, 259.576561},
  {71.613253, 48.381239, 272.408705, 210.842156, 259.198965, 196.854525, 258.610581},
  {70.550394, 49.445061, 270.681145, 210.879901, 257.788462, 196.880890, 257.622031},
  {69.539995, 50.456289, 269.119074, 210.877195, 256.397665, 196.878850, 256.612907},
  {68.585813, 51.411177, 267.713275, 210.839017, 255.026480, 196.850166, 255.585084},
  {67.688316, 52.309270, 266.454458, 210.769897, 253.674479, 196.796694, 254.540266},
  {66.845681, 53.152406, 265.333630, 210.673940, 252.341014, 196.720346, 253.479963},
  {66.054563, 53.943940, 264.342350, 210.554861, 251.025300, 196.623059, 252.405480},
  {65.310688, 54.688160, 263.472876, 210.416024, 249.726486, 196.506770, 251.317916},
  {64.609259, 55.389874, 262.718253, 210.260493, 248.443695, 196.373404, 250.218176},
  {63.945242, 56.054123, 262.072353, 210.091062, 247.176063, 196.224860, 249.106978},
  {63.313548, 56.686004, 261.529879, 209.910305, 245.922760, 196.063006, 247.984871},
  {62.709142, 57.290558, 261.086361, 209.720605, 244.683006, 195.889683, 246.852244},
  {62.127102, 57.872714, 260.738134, 209.524188, 243.456087, 195.706705, 245.709345},
  {61.421903, 58.302442, 260.482802, 209.323617, 242.244134, 195.516313, 244.558944},
  {60.391402, 58.381891, 260.318322, 209.122052, 241.053243, 195.321449, 243.407682},
  {58.980483, 58.041731, 260.241484, 208.922751, 239.890592, 195.125319, 242.263536},
  {57.198367, 57.272745, 260.247572, 208.728781, 238.763189, 194.931097, 241.134667},
  {55.054156, 56.065835, 260.330379, 208.542930, 237.677904, 194.741833, 240.029438},
  {52.691835, 54.552579, 260.482683, 208.367192, 236.638824, 194.559892, 238.953664},
  {50.310443, 52.933941, 260.697419, 208.202580, 235.646171, 194.386702, 237.909247},
  {47.968973, 51.275381, 260.968053, 208.049643, 234.699060, 194.223230, 236.896852},
  {45.665248, 49.579074, 261.288404, 207.908708, 233.796695, 194.070222, 235.917147},
  {43.397706, 47.846585, 261.652550, 207.779899, 232.938352, 193.928216, 234.970811},
  {41.165278, 46.078988, 262.054744, 207.663167, 232.123364, 193.797559, 234.058540},
  {38.967272, 44.276974, 262.489350, 207.558309, 231.351105, 193.678420, 233.181051},
  {36.803280, 42.440951, 262.950801, 207.464982, 230.620980, 193.570806, 232.339082},
  {34.673085, 40.571138, 263.433565, 207.382732, 229.932413, 193.474573, 231.533386},
  {32.576573, 38.667648, 263.932127, 207.311000, 229.284842, 193.389437, 230.764733},
  {30.513659, 36.730566, 264.440990, 207.249149, 228.677711, 193.314992, 230.033904},
  {28.484213, 34.760020, 264.954673, 207.196474, 228.110468, 193.250718, 229.341681},
  {26.488001, 32.756246, 265.467738, 207.152221, 227.582559, 193.195999, 228.688845},
  {24.524625, 30.719640, 265.974809, 207.115604, 227.093435, 193.150138, 228.076166},
  {22.593483, 28.650803, 266.470608, 207.085817, 226.642549, 193.112370, 227.504396},
  {20.693732, 26.550576, 266.949999, 207.062052, 226.229357, 193.081883, 226.974261},
  {18.824266, 24.420065, 267.408023, 207.043511, 225.853328, 193.057834, 226.486452},
  {16.983702, 22.260651, 267.839948, 207.029420, 225.513947, 193.039368, 226.041622},
  {15.170377, 20.073996, 268.241315, 207.019040, 225.210717, 193.025635, 225.640377},
  {13.382355, 17.862037, 268.607983, 207.011677, 224.943171, 193.015809, 225.283271},
  {11.617439, 15.626969, 268.936168, 207.006695, 224.710876, 193.009108, 224.970804},
  {9.873197, 13.371224, 269.222486, 207.003520, 224.513437, 193.004809, 224.703414},
  {8.146981, 11.097451, 269.463985, 207.001651, 224.350508, 193.002264, 224.481480},
  {6.435959, 8.808479, 269.658179, 207.000665, 224.221793, 193.000914, 224.305315},
  {4.737147, 6.507295, 269.803067, 207.000219, 224.127051, 193.000302, 224.175171},
  {3.163284, 4.356716, 269.900751, 207.000055, 224.063786, 193.000076, 224.088037},
  {1.878940, 2.592172, 269.959130, 207.000009, 224.026207, 193.000013, 224.036193},
  {0.933358, 1.288865, 269.988228, 207.000001, 224.007540, 193.000001, 224.010416},
  {0.324729, 0.448604, 269.998368, 207.000000, 224.001045, 193.000000, 224.001444},
  {0.052251, 0.072193, 270.000000, 207.000000, 224.000000, 193.000000, 224.000000},
  {0.000000, 0.000000, 270.000000, 207.000000, 224.000000, 193.000000, 224.000000}};
}}
