/* Red43 Feed Switch from Front Start 4 aud */
//----------------------------------------------------------------------------
//
//  $Workfile: Red43Path60InPerSec.java$
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
//   Max Jerk  = 1000.000000
package org.usfirst.frc4089.Stealth2018.MPPaths;

public class Red43Path60InPerSec extends Path {
    public Red43Path60InPerSec() {
       kNumPoints =210;
       kPoints = new double[][]{
  {0.400000, 0.400000, 359.998252, 16.002611, 105.500000, 16.002184, 91.500000},
  {0.871310, 0.728690, 359.986578, 16.020037, 105.499998, 16.016757, 91.499998},
  {1.960602, 1.639398, 359.960287, 16.059249, 105.499979, 16.049545, 91.499983},
  {3.486045, 2.913955, 359.913461, 16.128970, 105.499903, 16.107824, 91.499919},
  {5.448309, 4.551691, 359.840072, 16.237936, 105.499668, 16.198858, 91.499723},
  {7.630438, 6.369560, 359.736867, 16.390543, 105.499105, 16.326248, 91.499253},
  {9.815403, 8.184593, 359.603384, 16.586848, 105.497975, 16.489937, 91.498310},
  {12.004056, 9.995936, 359.439017, 16.826921, 105.495969, 16.689849, 91.496640},
  {14.197284, 11.802704, 359.243018, 17.110848, 105.492704, 16.925888, 91.493926},
  {16.396013, 13.603967, 359.014486, 17.438730, 105.487719, 17.197936, 91.489790},
  {18.601218, 15.398753, 358.752360, 17.810684, 105.480471, 17.505852, 91.483790},
  {20.813930, 17.186028, 358.455412, 18.226839, 105.470331, 17.849471, 91.475418},
  {23.035247, 18.964694, 358.122232, 18.687339, 105.456577, 18.228596, 91.464095},
  {25.266339, 20.733581, 357.751220, 19.192338, 105.438388, 18.642999, 91.449170},
  {27.508460, 22.491433, 357.340570, 19.742003, 105.414836, 19.092415, 91.429914},
  {29.762954, 24.236906, 356.888256, 20.336508, 105.384877, 19.576539, 91.405519},
  {32.031262, 25.968556, 356.392015, 20.976032, 105.347341, 20.095018, 91.375089},
  {34.314934, 27.684834, 355.849332, 21.660759, 105.300921, 20.647446, 91.337640},
  {36.615626, 29.384080, 355.257419, 22.390869, 105.244158, 21.233360, 91.292091},
  {38.935106, 31.064526, 354.613200, 23.166532, 105.175432, 21.852227, 91.237261},
  {41.275248, 32.724292, 353.913290, 23.987905, 105.092937, 22.503437, 91.171861},
  {43.638027, 34.361403, 353.153983, 24.855116, 104.994672, 23.186295, 91.094491},
  {46.025492, 35.973803, 352.331234, 25.768255, 104.878417, 23.900011, 91.003632},
  {48.439740, 37.559394, 351.440656, 26.727356, 104.741713, 24.643684, 90.897641},
  {50.882861, 39.116079, 350.477521, 27.732377, 104.581840, 25.416293, 90.774749},
  {53.356864, 40.641843, 349.436768, 28.783172, 104.395796, 26.216684, 90.633051},
  {55.863569, 42.134860, 348.313040, 29.879459, 104.180272, 27.043556, 90.470507},
  {58.404461, 43.593637, 347.100735, 31.020779, 103.931635, 27.895452, 90.284938},
  {60.980491, 45.017211, 345.794094, 32.206446, 103.645906, 28.770744, 90.074025},
  {63.591823, 46.405414, 344.387332, 33.435487, 103.318756, 29.667628, 89.835313},
  {66.004514, 47.592211, 342.880221, 34.702113, 102.946874, 30.580929, 89.567193},
  {67.977122, 48.419091, 341.279322, 35.995728, 102.528644, 31.502361, 89.269321},
  {69.495748, 48.899980, 339.593475, 37.305436, 102.063319, 32.423933, 88.941927},
  {70.545008, 49.050293, 337.834040, 38.620118, 101.551213, 33.338048, 88.585885},
  {71.108843, 48.886117, 336.015008, 39.928541, 100.993884, 34.237578, 88.202756},
  {71.410068, 48.584614, 334.146637, 41.223815, 100.392200, 35.118843, 87.793418},
  {71.677220, 48.317210, 332.234505, 42.503302, 99.745700, 35.981348, 87.357636},
  {71.903588, 48.090625, 330.285294, 43.764246, 99.054272, 36.824703, 86.895212},
  {72.082880, 47.911156, 328.306713, 45.003829, 98.318196, 37.648622, 86.405978},
  {72.209576, 47.784335, 326.307378, 46.219243, 97.538158, 38.452921, 85.889801},
  {72.279254, 47.714587, 324.296630, 47.407758, 96.715267, 39.237512, 85.346579},
  {72.288886, 47.704945, 322.284303, 48.566800, 95.851037, 40.002387, 84.776253},
  {72.237043, 47.756841, 320.280469, 49.694027, 94.947358, 40.747606, 84.178813},
  {72.123995, 47.870000, 318.295153, 50.787394, 94.006456, 41.473284, 83.554309},
  {71.951699, 48.042466, 316.338061, 51.845205, 93.030825, 42.179577, 82.902862},
  {71.723656, 48.270730, 314.418323, 52.866152, 92.023165, 42.866668, 82.224680},
  {71.444681, 48.549969, 312.544282, 53.849337, 90.986302, 43.534767, 81.520064},
  {71.120590, 48.874359, 310.723326, 54.794267, 89.923117, 44.184104, 80.789419},
  {70.757858, 49.237415, 308.961785, 55.700847, 88.836478, 44.814929, 80.033252},
  {70.363272, 49.632342, 307.264873, 56.569346, 87.729182, 45.427518, 79.252174},
  {69.943612, 50.052350, 305.636696, 57.400362, 86.603901, 46.022173, 78.446891},
  {69.505382, 50.490928, 304.080294, 58.194775, 85.463153, 46.599232, 77.618195},
  {69.054605, 50.942047, 302.597720, 58.953703, 84.309271, 47.159070, 76.766950},
  {68.596678, 51.400304, 301.190142, 59.678451, 83.144392, 47.702103, 75.894075},
  {68.136295, 51.861002, 299.857961, 60.370466, 81.970452, 48.228794, 75.000530},
  {67.677413, 52.320181, 298.600930, 61.031301, 80.789185, 48.739648, 74.087300},
  {67.223266, 52.774604, 297.418269, 61.662574, 79.602138, 49.235213, 73.155378},
  {66.776408, 53.221718, 296.308786, 62.265942, 78.410676, 49.716083, 72.205755},
  {66.338768, 53.659591, 295.270967, 62.843074, 77.215999, 50.182888, 71.239404},
  {65.911728, 54.086845, 294.303075, 63.395629, 76.019160, 50.636293, 70.257274},
  {65.496191, 54.502575, 293.403226, 63.925247, 74.821076, 51.076996, 69.260282},
  {65.092660, 54.906282, 292.569453, 64.433532, 73.622548, 51.505722, 68.249306},
  {64.701299, 55.297798, 291.799760, 64.922044, 72.424275, 51.923220, 67.225180},
  {64.322004, 55.677233, 291.092171, 65.392297, 71.226865, 52.330259, 66.188695},
  {63.954450, 56.044912, 290.444763, 65.845756, 70.030854, 52.727625, 65.140595},
  {63.598139, 56.401332, 289.855694, 66.283831, 68.836710, 53.116116, 64.081577},
  {63.252443, 56.747126, 289.323225, 66.707883, 67.644850, 53.496546, 63.012293},
  {62.916628, 57.083024, 288.845736, 67.119220, 66.455648, 53.869736, 61.933350},
  {62.589890, 57.409836, 288.421742, 67.519104, 65.269439, 54.236518, 60.845312},
  {62.271369, 57.728421, 288.049896, 67.908749, 64.086533, 54.597730, 59.748702},
  {61.960164, 58.039679, 287.729000, 68.289325, 62.907217, 54.954220, 58.644005},
  {61.655351, 58.344537, 287.458005, 68.661964, 61.731763, 55.306845, 57.531669},
  {61.355988, 58.643937, 287.236021, 69.027759, 60.560432, 55.656467, 56.412112},
  {61.061122, 58.938832, 287.062309, 69.387769, 59.393480, 56.003962, 55.285719},
  {60.769795, 59.230181, 286.936290, 69.743023, 58.231162, 56.350213, 54.152848},
  {60.481044, 59.518946, 286.857541, 70.094521, 57.073738, 56.696119, 53.013835},
  {60.193906, 59.806092, 286.825798, 70.443240, 55.921471, 57.042590, 51.868992},
  {59.907418, 60.092582, 286.840954, 70.790133, 54.774639, 57.390556, 50.718615},
  {59.620614, 60.379380, 286.903060, 71.136134, 53.633530, 57.740961, 49.562983},
  {59.332532, 60.667450, 287.012324, 71.482160, 52.498450, 58.094774, 48.402366},
  {59.042206, 60.957756, 287.169114, 71.829113, 51.369727, 58.452986, 47.237024},
  {58.748675, 61.251261, 287.373954, 72.177883, 50.247709, 58.816616, 46.067212},
  {58.450977, 61.548925, 287.627525, 72.529346, 49.132774, 59.186712, 44.893186},
  {58.148158, 61.851702, 287.930664, 72.884371, 48.025327, 59.564354, 43.715205},
  {57.839269, 62.160540, 288.284365, 73.243817, 46.925803, 59.950661, 42.533536},
  {57.523379, 62.476370, 288.689774, 73.608533, 45.834677, 60.346788, 41.348461},
  {57.199578, 62.800102, 289.148185, 73.979361, 44.752455, 60.753934, 40.160281},
  {56.866987, 63.132612, 289.661035, 74.357137, 43.679689, 61.173343, 38.969320},
  {56.524780, 63.474727, 290.229898, 74.742684, 42.616969, 61.606306, 37.775939},
  {56.172196, 63.827206, 290.856472, 75.136820, 41.564932, 62.054167, 36.580537},
  {55.808562, 64.190721, 291.542565, 75.540348, 40.524257, 62.518317, 35.383564},
  {55.433332, 64.565817, 292.290075, 75.954059, 39.495673, 63.000203, 34.185531},
  {55.046112, 64.952887, 293.100962, 76.378728, 38.479954, 63.501320, 32.987018},
  {54.646712, 65.352119, 293.977220, 76.815112, 37.477919, 64.023213, 31.788691},
  {54.235195, 65.763449, 294.920831, 77.263945, 36.490431, 64.567473, 30.591313},
  {53.811935, 66.186503, 295.933716, 77.725934, 35.518394, 65.135725, 29.395759},
  {53.377684, 66.620526, 297.017674, 78.201756, 34.562746, 65.729626, 28.203031},
  {52.933647, 67.064315, 298.174305, 78.692056, 33.624452, 66.350842, 27.014275},
  {52.481550, 67.516143, 299.404926, 79.197439, 32.704499, 67.001036, 25.830798},
  {52.023720, 67.973684, 300.710477, 79.718469, 31.803881, 67.681845, 24.654079},
  {51.563144, 68.433951, 302.091405, 80.255669, 30.923586, 68.394847, 23.485785},
  {51.103525, 68.893245, 303.547552, 80.809515, 30.064585, 69.141531, 22.327780},
  {50.649300, 69.347132, 305.078036, 81.380440, 29.227814, 69.923259, 21.182132},
  {50.205629, 69.790457, 306.681128, 81.968833, 28.414158, 70.741219, 20.051103},
  {49.778331, 70.217405, 308.354149, 82.575044, 27.624436, 71.596380, 18.937150},
  {49.373769, 70.621623, 310.093376, 83.199388, 26.859385, 72.489446, 17.842893},
  {48.998661, 70.996400, 311.893990, 83.842150, 26.119652, 73.420808, 16.771090},
  {48.659835, 71.334917, 313.750052, 84.503588, 25.405781, 74.390501, 15.724589},
  {47.838338, 72.106268, 315.736508, 85.176618, 24.725757, 75.405190, 14.699830},
  {46.003537, 73.988471, 318.027259, 85.847986, 24.096634, 76.485108, 13.688151},
  {44.763365, 75.227165, 320.520952, 86.526216, 23.512248, 77.625072, 12.706249},
  {43.452686, 76.536144, 323.229122, 87.209635, 22.975413, 78.829004, 11.760912},
  {42.099777, 77.887152, 326.158678, 87.896543, 22.488469, 80.100016, 10.860306},
  {40.748189, 79.236689, 329.309402, 88.585456, 22.053078, 81.439831, 10.013974},
  {39.458282, 80.524501, 332.671200, 89.275440, 21.670058, 82.848093, 9.232645},
  {38.305746, 81.675049, 336.221593, 89.966485, 21.339317, 84.321679, 8.527754},
  {37.375589, 82.603522, 339.924194, 90.659837, 21.059967, 85.854153, 7.910617},
  {36.751031, 83.226909, 343.728996, 91.358152, 20.830597, 87.435619, 7.391336},
  {36.498882, 83.478576, 347.575058, 92.065357, 20.649687, 89.053111, 6.977585},
  {36.655409, 83.322348, 351.395507, 92.786174, 20.516005, 90.691594, 6.673580},
  {37.217808, 82.761009, 355.123926, 93.525411, 20.428855, 92.335399, 6.479523},
  {38.144798, 81.835709, 358.700657, 94.287216, 20.388073, 93.969754, 6.391672},
  {39.366137, 80.616490, 2.077533, 95.074518, 20.393803, 95.582044, 6.403005},
  {40.797197, 79.187757, 5.220238, 95.888780, 20.446161, 97.162561, 6.504228},
  {42.353479, 77.633816, 8.108278, 96.730073, 20.544916, 98.704693, 6.684870},
  {43.961255, 76.028250, 10.733227, 97.597365, 20.689285, 100.204675, 6.934216},
  {45.563027, 74.428469, 13.096064, 98.488900, 20.877862, 101.661081, 7.241980},
  {47.118529, 72.874702, 15.204354, 99.402571, 21.108660, 103.074246, 7.598708},
  {48.602883, 71.391818, 17.069736, 100.336214, 21.379226, 104.445710, 7.995951},
  {50.003500, 69.992422, 18.705907, 101.287821, 21.686771, 105.777771, 8.426290},
  {51.316809, 68.680115, 20.127150, 102.255666, 22.028301, 107.073131, 8.883263},
  {52.545397, 67.452336, 21.347323, 103.238370, 22.400727, 108.334658, 9.361254},
  {53.695784, 66.302594, 22.379218, 104.234926, 22.800940, 109.565216, 9.855362},
  {54.776816, 65.222070, 23.234182, 105.244694, 23.225872, 110.767557, 10.361270},
  {55.798578, 64.200701, 23.921909, 106.267386, 23.672519, 111.944263, 10.875134},
  {56.771708, 63.227866, 24.450354, 107.303040, 24.137957, 113.097705, 11.393474},
  {57.707004, 62.292781, 24.825706, 108.351997, 24.619339, 114.230027, 11.913090},
  {58.615221, 61.384700, 25.052391, 109.414884, 25.113878, 115.343139, 12.430985},
  {59.507006, 60.492983, 25.133094, 110.492594, 25.618828, 116.438708, 12.944297},
  {60.392911, 59.607082, 25.068773, 111.586277, 26.131448, 117.518158, 13.450250},
  {57.085685, 62.795513, 25.536132, 112.619630, 26.616929, 118.654753, 13.984539},
  {54.019158, 65.979383, 26.515101, 113.590527, 27.090836, 119.840598, 14.563401},
  {53.449399, 66.548851, 27.587322, 114.542647, 27.576842, 121.026046, 15.168557},
  {52.794828, 67.203054, 28.766672, 115.473507, 28.075256, 122.210921, 15.803042},
  {52.038893, 67.958522, 30.069740, 116.380199, 28.586264, 123.394951, 16.470438},
  {51.161335, 68.835479, 31.516426, 117.259311, 29.109862, 124.577713, 17.174997},
  {50.137425, 69.858608, 33.130681, 118.106844, 29.645764, 125.758551, 17.921798},
  {48.937209, 71.057800, 34.941351, 118.918124, 30.193273, 126.936450, 18.716931},
  {47.524966, 72.468687, 36.983131, 119.687718, 30.751102, 128.109836, 19.567725},
  {45.859332, 74.132514, 39.297480, 120.409405, 31.317141, 129.276261, 20.482988},
  {43.894994, 76.094431, 41.933272, 121.076211, 31.888173, 130.431917, 21.473242},
  {41.587627, 78.398550, 44.946635, 121.680632, 32.459564, 131.570902, 22.550853},
  {38.904890, 81.076966, 48.398990, 122.215131, 33.025026, 132.684141, 23.729874},
  {35.847418, 84.128798, 52.351660, 122.673080, 33.576658, 133.757924, 25.025270},
  {32.483467, 87.485658, 56.854811, 123.050199, 34.105668, 134.772227, 26.450993},
  {28.995246, 90.965544, 61.928809, 123.346316, 34.604270, 135.699406, 28.016314},
  {25.718716, 94.233327, 67.539049, 123.566644, 35.069067, 136.504606, 29.720316},
  {23.133381, 96.811128, 73.572444, 123.721219, 35.505150, 137.149712, 31.545911},
  {21.751643, 98.188606, 79.832007, 123.821460, 35.928476, 137.601582, 33.456987},
  {21.916610, 98.024154, 86.064567, 123.874996, 36.363527, 137.841984, 35.402675},
  {23.628204, 96.317786, 92.016968, 123.882222, 36.836036, 137.873548, 37.328772},
  {26.541954, 93.412366, 97.492471, 123.837266, 37.364968, 137.717734, 39.190511},
  {30.142130, 89.821507, 102.378778, 123.732187, 37.958582, 137.406712, 40.959812},
  {33.945756, 86.026563, 106.642636, 123.561032, 38.615569, 136.974568, 42.625188},
  {37.615374, 82.364195, 110.306002, 123.321685, 39.328786, 136.451621, 44.187261},
  {40.969965, 79.015269, 113.420437, 123.015707, 40.088913, 135.862288, 45.653566},
  {43.944630, 76.044858, 116.048108, 122.647248, 40.886842, 135.225207, 47.034601},
  {46.543400, 73.449215, 118.250515, 122.221868, 41.714831, 134.554278, 48.341417},
  {48.803913, 71.190975, 120.082999, 121.745648, 42.566854, 133.859850, 49.584410},
  {50.776069, 69.220460, 121.592737, 121.224628, 43.438532, 133.149735, 50.772823},
  {52.510947, 67.486764, 122.818547, 120.664510, 44.326917, 132.429987, 51.914641},
  {54.055933, 65.942625, 123.791498, 120.070522, 45.230242, 131.705460, 53.016654},
  {55.453194, 64.545961, 124.535756, 119.447403, 46.147709, 130.980218, 54.084595},
  {56.739783, 63.259783, 125.069428, 118.799426, 47.079313, 130.257816, 55.123274},
  {57.930916, 62.032951, 125.405184, 118.130670, 48.025443, 129.541725, 56.136412},
  {58.855850, 60.636237, 125.550911, 117.447219, 48.983827, 128.837608, 57.123792},
  {59.341228, 58.878969, 125.513074, 116.757234, 49.949473, 128.152996, 58.081915},
  {59.383087, 56.765124, 125.298791, 116.068900, 50.917325, 127.494997, 59.007090},
  {58.974921, 54.301227, 124.916243, 115.390320, 51.882076, 126.870175, 59.895374},
  {58.125382, 51.514609, 124.375142, 114.729207, 52.838295, 126.284227, 60.742821},
  {57.037865, 48.601894, 123.684644, 114.090592, 53.783545, 125.740031, 61.548245},
  {55.910042, 45.729461, 122.851345, 113.476983, 54.718348, 125.238114, 62.312806},
  {54.747763, 42.891469, 121.880883, 112.890614, 55.643064, 124.778685, 63.037234},
  {53.553211, 40.085744, 120.778538, 112.333496, 56.557831, 124.361619, 63.721926},
  {52.325090, 37.313591, 119.549807, 111.807450, 57.462508, 123.986432, 64.367028},
  {51.058734, 34.579687, 118.200947, 111.314122, 58.356614, 123.652261, 64.972529},
  {49.746246, 31.891940, 116.739513, 110.854986, 59.239263, 123.357844, 65.538353},
  {48.376736, 29.261250, 115.174840, 110.431316, 60.109106, 123.101511, 66.064453},
  {46.936755, 26.701080, 113.518470, 110.044145, 60.964280, 122.881186, 66.550906},
  {45.410987, 24.226755, 111.784451, 109.694209, 61.802378, 122.694421, 66.998000},
  {43.783228, 21.854487, 109.989485, 109.381869, 62.620444, 122.538444, 67.406312},
  {42.037631, 19.600129, 108.152872, 109.107032, 63.415007, 122.410233, 67.776755},
  {40.160109, 17.477768, 106.296216, 108.869080, 64.182152, 122.306613, 68.110599},
  {38.139729, 15.498331, 104.442912, 108.666813, 64.917641, 122.224366, 68.409454},
  {35.969888, 13.668411, 102.617434, 108.498430, 65.617055, 122.160334, 68.675218},
  {33.649060, 11.989518, 100.844505, 108.361543, 66.275968, 122.111522, 68.909987},
  {31.180983, 10.457895, 99.148235, 108.253247, 66.890112, 122.075171, 69.115962},
  {28.574250, 9.064930, 97.551324, 108.170226, 67.455535, 122.048811, 69.295334},
  {25.841397, 7.798070, 96.074417, 108.108893, 67.968711, 122.030287, 69.450192},
  {22.997667, 6.642057, 94.735662, 108.065553, 68.426617, 122.017760, 69.582441},
  {20.059682, 5.580260, 93.550484, 108.036559, 68.826762, 122.009687, 69.693754},
  {17.044216, 4.595898, 92.531562, 108.018469, 69.167166, 122.004806, 69.785542},
  {13.967247, 3.672992, 91.688958, 108.008180, 69.446321, 122.002098, 69.858952},
  {10.843343, 2.796981, 91.030350, 108.003033, 69.663127, 122.000770, 69.914876},
  {7.714099, 1.962233, 90.559552, 108.000892, 69.817394, 122.000225, 69.954117},
  {4.910642, 1.237630, 90.258912, 108.000191, 69.915605, 122.000048, 69.978869},
  {2.734736, 0.685462, 90.091176, 108.000024, 69.970299, 122.000006, 69.992578},
  {1.193741, 0.298378, 90.017890, 108.000001, 69.994174, 122.000000, 69.998545},
  {0.291304, 0.072736, 90.000000, 108.000000, 70.000000, 122.000000, 70.000000},
  {0.000000, 0.000000, 90.000000, 108.000000, 70.000000, 122.000000, 70.000000}};
}}
