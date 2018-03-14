//----------------------------------------------------------------------------
//
//  $Workfile: Red12Path60InPerSec.java
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

public class Red12Path60InPerSec extends Path {
    public Red12Path60InPerSec() {
       kNumPoints = 314;
       kPoints = new double[][]{
  {0.400000, 0.400000, 0.004559, 16.002742, 287.250000, 16.003856, 273.250000},
  {0.664947, 0.935053, 0.026667, 16.016041, 287.250004, 16.022557, 273.250005},
  {1.496180, 2.103820, 0.076403, 16.045965, 287.250031, 16.064633, 273.250043},
  {2.660048, 3.739952, 0.164795, 16.099165, 287.250143, 16.139432, 273.250201},
  {4.156781, 5.843216, 0.302831, 16.182300, 287.250482, 16.256296, 273.250677},
  {5.820472, 8.179521, 0.495922, 16.298707, 287.251293, 16.419882, 273.251818},
  {7.485245, 10.514740, 0.743890, 16.448403, 287.252913, 16.630165, 273.254093},
  {9.151557, 12.848417, 1.046482, 16.631412, 287.255773, 16.887102, 273.258108},
  {10.819973, 15.179985, 1.403355, 16.847762, 287.260399, 17.190632, 273.264598},
  {12.491193, 17.508743, 1.814048, 17.097487, 287.267413, 17.540669, 273.274430},
  {14.166072, 19.833835, 2.277962, 17.380628, 287.277529, 17.937093, 273.288593},
  {15.845639, 22.154232, 2.794329, 17.697230, 287.291554, 18.379743, 273.308200},
  {17.531120, 24.468708, 3.362181, 18.047347, 287.310384, 18.868411, 273.334481},
  {19.223945, 26.775832, 3.980314, 18.431037, 287.335006, 19.402829, 273.368774},
  {20.925762, 29.073956, 4.647256, 18.848366, 287.366490, 19.982660, 273.412516},
  {22.638440, 31.361211, 5.361230, 19.299408, 287.405990, 20.607493, 273.467234},
  {24.364062, 33.635514, 6.120114, 19.784245, 287.454738, 21.276829, 273.534530},
  {26.104915, 35.894580, 6.921415, 20.302965, 287.514039, 21.990075, 273.616065},
  {27.863462, 38.135946, 7.762237, 20.855663, 287.585264, 22.746539, 273.713545},
  {29.642314, 40.357003, 8.639255, 21.442445, 287.669844, 23.545423, 273.828692},
  {31.444183, 42.555040, 9.548700, 22.063421, 287.769262, 24.385823, 273.963232},
  {33.271828, 44.727301, 10.486354, 22.718708, 287.885039, 25.266727, 274.118863},
  {35.127996, 46.871043, 11.447547, 23.408432, 288.018726, 26.187022, 274.297231},
  {37.015351, 48.983602, 12.427173, 24.132721, 288.171889, 27.145500, 274.499905},
  {38.936410, 51.062464, 13.419717, 24.891714, 288.346094, 28.140871, 274.728349},
  {40.893475, 53.105333, 14.419284, 25.685554, 288.542891, 29.171776, 274.983899},
  {42.888574, 55.110181, 15.419649, 26.514393, 288.763793, 30.236807, 275.267733},
  {44.923414, 57.075305, 16.414307, 27.378394, 289.010262, 31.334528, 275.580853},
  {46.999352, 58.999350, 17.396532, 28.277737, 289.283681, 32.463500, 275.924063},
  {49.117386, 60.881319, 18.359435, 29.212621, 289.585341, 33.622301, 276.297951},
  {51.097572, 62.501172, 19.292842, 30.179867, 289.915220, 34.805418, 276.701428},
  {52.752705, 63.646121, 20.184489, 31.172892, 290.271648, 36.003510, 277.131438},
  {54.071221, 64.327720, 21.024003, 32.185100, 290.652323, 37.207726, 277.584300},
  {55.040916, 64.558162, 21.803006, 33.209885, 291.054340, 38.409716, 278.055811},
  {55.649381, 64.349846, 22.515153, 34.240622, 291.474248, 39.601610, 278.541352},
  {56.071976, 63.927394, 23.158131, 35.274110, 291.909595, 40.779893, 279.037674},
  {56.491444, 63.508053, 23.732451, 36.310614, 292.359233, 41.945142, 279.543146},
  {56.907432, 63.092177, 24.238681, 37.350433, 292.822006, 43.097975, 280.056202},
  {57.319824, 62.679883, 24.677409, 38.393908, 293.296752, 44.239032, 280.575332},
  {57.728703, 62.271086, 25.049209, 39.441420, 293.782304, 45.368971, 281.099081},
  {58.134325, 61.865533, 25.354612, 40.493385, 294.277491, 46.488456, 281.626044},
  {58.537079, 61.462833, 25.594089, 41.550255, 294.781139, 47.598153, 282.154860},
  {58.937465, 61.062489, 25.768024, 42.612518, 295.292066, 48.698718, 282.684205},
  {59.336065, 60.663917, 25.876711, 43.680694, 295.809083, 49.790800, 283.212790},
  {59.733518, 60.266479, 25.920334, 44.755333, 296.330992, 50.875028, 283.739354},
  {60.130501, 59.869498, 25.898971, 45.837013, 296.856578, 51.952011, 284.262659},
  {60.527706, 59.472283, 25.812583, 46.926334, 297.384612, 53.022338, 284.781488},
  {60.925815, 59.074150, 25.661022, 48.023922, 297.913840, 54.086566, 285.294635},
  {61.325483, 58.674445, 25.444032, 49.130418, 298.442984, 55.145227, 285.800909},
  {61.727315, 58.272563, 25.161256, 50.246476, 298.970733, 56.198819, 286.299126},
  {62.131841, 57.867974, 24.812253, 51.372760, 299.495739, 57.247807, 286.788110},
  {62.539491, 57.460246, 24.396511, 52.509934, 300.016612, 58.292620, 287.266689},
  {62.950569, 57.049076, 23.913465, 53.658661, 300.531915, 59.333651, 287.733693},
  {63.365220, 56.634317, 23.362531, 54.819587, 301.040158, 60.371254, 288.187960},
  {63.783401, 56.216014, 22.743129, 55.993338, 301.539794, 61.405744, 288.628332},
  {64.204844, 55.794434, 22.054724, 57.180508, 302.029215, 62.437396, 289.053656},
  {64.629019, 55.370106, 21.296866, 58.381644, 302.506748, 63.466447, 289.462793},
  {65.055099, 54.943857, 20.469242, 59.597233, 302.970655, 64.493096, 289.854614},
  {65.481924, 54.516849, 19.571730, 60.827688, 303.419132, 65.517502, 290.228012},
  {65.907967, 54.090608, 18.604454, 62.073331, 303.850314, 66.539793, 290.581903},
  {66.331308, 53.667055, 17.567857, 63.334373, 304.262275, 67.560064, 290.915233},
  {66.749620, 53.248519, 16.462759, 64.610893, 304.653043, 68.578382, 291.226985},
  {67.160164, 52.837742, 15.290433, 65.902826, 305.020610, 69.594794, 291.516190},
  {67.559806, 52.437861, 14.052661, 67.209936, 305.362946, 70.609326, 291.781925},
  {67.945053, 52.052370, 12.751799, 68.531805, 305.678025, 71.621998, 292.023330},
  {68.312120, 51.685058, 11.390823, 69.867816, 305.963846, 72.632820, 292.239606},
  {68.657021, 51.339918, 9.973363, 71.217143, 306.218464, 73.641807, 292.430026},
  {68.975686, 51.021024, 8.503714, 72.578748, 306.440024, 74.648978, 292.593936},
  {69.264103, 50.732392, 6.986828, 73.951382, 306.626794, 75.654358, 292.730756},
  {69.518482, 50.477817, 5.428280, 75.333592, 306.777202, 76.657988, 292.839987},
  {69.735423, 50.260706, 3.834202, 76.723742, 306.889871, 77.659916, 292.921207},
  {69.912078, 50.083909, 2.211190, 78.120036, 306.963647, 78.660198, 292.974071},
  {70.046305, 49.949573, 0.566193, 79.520550, 306.997630, 79.658895, 292.998314},
  {61.698191, 58.111480, 0.272614, 80.754507, 307.001799, 80.821119, 293.001957},
  {57.503589, 62.496156, 0.681262, 81.904539, 307.011386, 82.070999, 293.012376},
  {57.552562, 62.447193, 1.081894, 83.055454, 307.029110, 83.319794, 293.031606},
  {57.601842, 62.397923, 1.474460, 84.207203, 307.054822, 84.567442, 293.059458},
  {57.651385, 62.348390, 1.858915, 85.359743, 307.088372, 85.813882, 293.095740},
  {57.701149, 62.298635, 2.235225, 86.513029, 307.129609, 87.059059, 293.140261},
  {57.751096, 62.248697, 2.603360, 87.667021, 307.178379, 88.302923, 293.192828},
  {57.801189, 62.198614, 2.963294, 88.821680, 307.234529, 89.545427, 293.253249},
  {57.851392, 62.148420, 3.315011, 89.976971, 307.297903, 90.786529, 293.321329},
  {57.901672, 62.098148, 3.658498, 91.132860, 307.368347, 92.026192, 293.396877},
  {57.951999, 62.047830, 3.993747, 92.289316, 307.445702, 93.264382, 293.479699},
  {58.002344, 61.997493, 4.320755, 93.446309, 307.529813, 94.501068, 293.569602},
  {58.052681, 61.947164, 4.639523, 94.603814, 307.620521, 95.736225, 293.666395},
  {58.102985, 61.896868, 4.950057, 95.761806, 307.717669, 96.969829, 293.769885},
  {58.153234, 61.846627, 5.252366, 96.920263, 307.821098, 98.201861, 293.879882},
  {58.203406, 61.796462, 5.546462, 98.079164, 307.930649, 99.432305, 293.996195},
  {58.253484, 61.746392, 5.832360, 99.238493, 308.046163, 100.661148, 294.118634},
  {58.303450, 61.696432, 6.110080, 100.398234, 308.167482, 101.888380, 294.247013},
  {58.353289, 61.646600, 6.379641, 101.558373, 308.294447, 103.113994, 294.381142},
  {58.402988, 61.596907, 6.641067, 102.718899, 308.426899, 104.337987, 294.520837},
  {58.452535, 61.547367, 6.894383, 103.879802, 308.564679, 105.560355, 294.665912},
  {58.501920, 61.497988, 7.139615, 105.041075, 308.707630, 106.781101, 294.816182},
  {58.551133, 61.448781, 7.376791, 106.202712, 308.855592, 108.000227, 294.971466},
  {58.600168, 61.399752, 7.605940, 107.364710, 309.008408, 109.217738, 295.131582},
  {58.649019, 61.350906, 7.827092, 108.527067, 309.165920, 110.433643, 295.296350},
  {58.697681, 61.302250, 8.040279, 109.689782, 309.327970, 111.647951, 295.465591},
  {58.746150, 61.253786, 8.245532, 110.852857, 309.494403, 112.860674, 295.639127},
  {58.794424, 61.205516, 8.442883, 112.016296, 309.665061, 114.071824, 295.816783},
  {58.842503, 61.157442, 8.632363, 113.180103, 309.839789, 115.281416, 295.998384},
  {58.890386, 61.109564, 8.814006, 114.344284, 310.018429, 116.489468, 296.183756},
  {58.938073, 61.061881, 8.987842, 115.508848, 310.200828, 117.695997, 296.372727},
  {58.985568, 61.014390, 9.153903, 116.673804, 310.386830, 118.901021, 296.565126},
  {59.032872, 60.967090, 9.312221, 117.839163, 310.576281, 120.104563, 296.760784},
  {59.079989, 60.919977, 9.462826, 119.004936, 310.769026, 121.306643, 296.959532},
  {59.126923, 60.873046, 9.605748, 120.171137, 310.964913, 122.507285, 297.161202},
  {59.173679, 60.826293, 9.741017, 121.337782, 311.163787, 123.706511, 297.365630},
  {59.220264, 60.779711, 9.868659, 122.504885, 311.365495, 124.904348, 297.572650},
  {59.266683, 60.733295, 9.988703, 123.672464, 311.569886, 126.100820, 297.782099},
  {59.312943, 60.687038, 10.101174, 124.840536, 311.776807, 127.295953, 297.993813},
  {59.359051, 60.640932, 10.206098, 126.009122, 311.986106, 128.489775, 298.207631},
  {59.405016, 60.594969, 10.303497, 127.178241, 312.197632, 129.682313, 298.423394},
  {59.450846, 60.549142, 10.393393, 128.347914, 312.411232, 130.873595, 298.640941},
  {59.496550, 60.503440, 10.475809, 129.518164, 312.626757, 132.063649, 298.860113},
  {59.542136, 60.457856, 10.550761, 130.689014, 312.844055, 133.252506, 299.080752},
  {59.587614, 60.412379, 10.618269, 131.860486, 313.062976, 134.440192, 299.302703},
  {59.632994, 60.367000, 10.678349, 133.032606, 313.283369, 135.626740, 299.525808},
  {59.678286, 60.321710, 10.731013, 134.205398, 313.505082, 136.812176, 299.749912},
  {59.723501, 60.276496, 10.776277, 135.378889, 313.727967, 137.996533, 299.974861},
  {59.768648, 60.231350, 10.814149, 136.553105, 313.951872, 139.179839, 300.200499},
  {59.813739, 60.186260, 10.844641, 137.728073, 314.176647, 140.362125, 300.426674},
  {59.858784, 60.141216, 10.867758, 138.903820, 314.402142, 141.543420, 300.653232},
  {59.903794, 60.096206, 10.883507, 140.080375, 314.628205, 142.723754, 300.880021},
  {59.948780, 60.051220, 10.891892, 141.257766, 314.854685, 143.903157, 301.106889},
  {59.993753, 60.006247, 10.892915, 142.436021, 315.081432, 145.081657, 301.333683},
  {60.038725, 59.961275, 10.886575, 143.615170, 315.308295, 146.259285, 301.560253},
  {60.083707, 59.916293, 10.872872, 144.795243, 315.535121, 147.436070, 301.786447},
  {60.128709, 59.871291, 10.851802, 145.976268, 315.761759, 148.612038, 302.012115},
  {60.173742, 59.826257, 10.823360, 147.158275, 315.988056, 149.787220, 302.237105},
  {60.218818, 59.781180, 10.787539, 148.341295, 316.213858, 150.961642, 302.461267},
  {60.263948, 59.736049, 10.744330, 149.525356, 316.439014, 152.135332, 302.684450},
  {60.309141, 59.690855, 10.693723, 150.710490, 316.663368, 153.308316, 302.906504},
  {60.354410, 59.645585, 10.635705, 151.896726, 316.886766, 154.480620, 303.127279},
  {60.399763, 59.600230, 10.570262, 153.084093, 317.109053, 155.652269, 303.346623},
  {60.445212, 59.554780, 10.497379, 154.272622, 317.330073, 156.823289, 303.564387},
  {60.490765, 59.509225, 10.417039, 155.462341, 317.549668, 157.993703, 303.780420},
  {60.536433, 59.463555, 10.329223, 156.653279, 317.767682, 159.163535, 303.994570},
  {60.582225, 59.417761, 10.233910, 157.845465, 317.983957, 160.332806, 304.206688},
  {60.628149, 59.371835, 10.131079, 159.038927, 318.198331, 161.501538, 304.416621},
  {60.674214, 59.325767, 10.020707, 160.233694, 318.410646, 162.669751, 304.624217},
  {60.720429, 59.279550, 9.902770, 161.429791, 318.620741, 163.837465, 304.829326},
  {60.766800, 59.233176, 9.777241, 162.627245, 318.828451, 165.004698, 305.031795},
  {60.813335, 59.186638, 9.644094, 163.826083, 319.033616, 166.171468, 305.231472},
  {60.860040, 59.139930, 9.503301, 165.026329, 319.236069, 167.337791, 305.428204},
  {60.906921, 59.093046, 9.354833, 166.228008, 319.435646, 168.503682, 305.621837},
  {60.953983, 59.045980, 9.198661, 167.431141, 319.632179, 169.669155, 305.812219},
  {61.001231, 58.998728, 9.034754, 168.635752, 319.825500, 170.834221, 305.999194},
  {61.048668, 58.951287, 8.863081, 169.841860, 320.015440, 171.998893, 306.182610},
  {61.096298, 58.903653, 8.683610, 171.049487, 320.201830, 173.163180, 306.362310},
  {61.144122, 58.855825, 8.496311, 172.258650, 320.384495, 174.327090, 306.538140},
  {61.192141, 58.807801, 8.301150, 173.469366, 320.563265, 175.490631, 306.709944},
  {61.240357, 58.759581, 8.098095, 174.681650, 320.737963, 176.653806, 306.877567},
  {61.288767, 58.711165, 7.887116, 175.895516, 320.908415, 177.816621, 307.040851},
  {61.337370, 58.662557, 7.668179, 177.110976, 321.074444, 178.979078, 307.199639},
  {61.386163, 58.613758, 7.441255, 178.328041, 321.235870, 180.141175, 307.353776},
  {61.435142, 58.564774, 7.206312, 179.546718, 321.392514, 181.302913, 307.503102},
  {61.484301, 58.515609, 6.963321, 180.767013, 321.544196, 182.464288, 307.647460},
  {61.533632, 58.466272, 6.712254, 181.988930, 321.690732, 183.625294, 307.786693},
  {61.583129, 58.416769, 6.453083, 183.212472, 321.831940, 184.785926, 307.920641},
  {61.632780, 58.367111, 6.185784, 184.437636, 321.967634, 185.946173, 308.049146},
  {61.682575, 58.317309, 5.910333, 185.664419, 322.097629, 187.106026, 308.172050},
  {61.732501, 58.267377, 5.626709, 186.892815, 322.221738, 188.265471, 308.289193},
  {61.782542, 58.217329, 5.334892, 188.122816, 322.339772, 189.424493, 308.400416},
  {61.832682, 58.167181, 5.034866, 189.354408, 322.451542, 190.583075, 308.505562},
  {61.882904, 58.116951, 4.726618, 190.587577, 322.556859, 191.741198, 308.604470},
  {61.933187, 58.066660, 4.410138, 191.822305, 322.655531, 192.898841, 308.696983},
  {61.983509, 58.016330, 4.085420, 193.058568, 322.747368, 194.055979, 308.782943},
  {62.033846, 57.965985, 3.752460, 194.296343, 322.832176, 195.212587, 308.862191},
  {62.084173, 57.915650, 3.411262, 195.535600, 322.909764, 196.368636, 308.934570},
  {62.134460, 57.865354, 3.061830, 196.776306, 322.979939, 197.524097, 308.999924},
  {62.184679, 57.815126, 2.704177, 198.018425, 323.042507, 198.678935, 309.058097},
  {62.234797, 57.765000, 2.338318, 199.261916, 323.097275, 199.833116, 309.108933},
  {62.284778, 57.715009, 1.964277, 200.506733, 323.144051, 200.986602, 309.152277},
  {62.334588, 57.665189, 1.582081, 201.752827, 323.182641, 202.139353, 309.187978},
  {62.384187, 57.615581, 1.191764, 203.000145, 323.212854, 203.291327, 309.215883},
  {62.433534, 57.566224, 0.793368, 204.248628, 323.234499, 204.442478, 309.235841},
  {62.482587, 57.517161, 0.386942, 205.498213, 323.247384, 205.592760, 309.247703},
  {62.531301, 57.468438, 359.972540, 206.748833, 323.251321, 206.742123, 309.251323},
  {62.579627, 57.420101, 359.550226, 208.000415, 323.246123, 207.890516, 309.246554},
  {62.627519, 57.372200, 359.120071, 209.252881, 323.231602, 209.037882, 309.233253},
  {62.674924, 57.324784, 358.682156, 210.506149, 323.207576, 210.184168, 309.211279},
  {62.721789, 57.277908, 358.236567, 211.760132, 323.173863, 211.329312, 309.180493},
  {62.768062, 57.231625, 357.783402, 213.014737, 323.130283, 212.473255, 309.140758},
  {62.813686, 57.185991, 357.322768, 214.269865, 323.076661, 213.615932, 309.091941},
  {62.858603, 57.141064, 356.854779, 215.525416, 323.012823, 214.757279, 309.033912},
  {62.902755, 57.096901, 356.379562, 216.781279, 322.938602, 215.897228, 308.966542},
  {62.946084, 57.053562, 355.897252, 218.037344, 322.853830, 217.035710, 308.889707},
  {62.988528, 57.011108, 355.407992, 219.293490, 322.758347, 218.172652, 308.803286},
  {63.030026, 56.969599, 354.911937, 220.549597, 322.651997, 219.307982, 308.707163},
  {63.070519, 56.929097, 354.409254, 221.805535, 322.534627, 220.441625, 308.601223},
  {63.109943, 56.889662, 353.900115, 223.061172, 322.406092, 221.573503, 308.485358},
  {63.148239, 56.851357, 353.384706, 224.316371, 322.266250, 222.703539, 308.359461},
  {63.185345, 56.814241, 352.863223, 225.570990, 322.114968, 223.831652, 308.223434},
  {63.221201, 56.778375, 352.335868, 226.824883, 321.952115, 224.957762, 308.077179},
  {63.255749, 56.743818, 351.802858, 228.077899, 321.777572, 226.081785, 307.920605},
  {63.288931, 56.710628, 351.264414, 229.329885, 321.591222, 227.203639, 307.753625},
  {63.320691, 56.678859, 350.720771, 230.580683, 321.392958, 228.323239, 307.576159},
  {63.350974, 56.648567, 350.172169, 231.830132, 321.182681, 229.440498, 307.388129},
  {63.379730, 56.619803, 349.618860, 233.078067, 320.960297, 230.555332, 307.189465},
  {63.406910, 56.592616, 349.061100, 234.324321, 320.725723, 231.667652, 306.980101},
  {63.432466, 56.567053, 348.499156, 235.568725, 320.478883, 232.777372, 306.759978},
  {63.456356, 56.543156, 347.933301, 236.811107, 320.219710, 233.884404, 306.529041},
  {63.478540, 56.520966, 347.363813, 238.051293, 319.948145, 234.988659, 306.287242},
  {63.498983, 56.500517, 346.790978, 239.289110, 319.664139, 236.090051, 306.034538},
  {63.517652, 56.481843, 346.215087, 240.524380, 319.367651, 237.188491, 305.770892},
  {63.534520, 56.464970, 345.636434, 241.756927, 319.058650, 238.283892, 305.496275},
  {63.549562, 56.449923, 345.055318, 242.986575, 318.737115, 239.376166, 305.210661},
  {63.562761, 56.436721, 344.472042, 244.213146, 318.403031, 240.465226, 304.914032},
  {63.574101, 56.425377, 343.886908, 245.436466, 318.056396, 241.550987, 304.606376},
  {63.583574, 56.415902, 343.300223, 246.656358, 317.697216, 242.633363, 304.287685},
  {63.591174, 56.408299, 342.712294, 247.872650, 317.325506, 243.712270, 303.957961},
  {63.596902, 56.402570, 342.123428, 249.085170, 316.941289, 244.787625, 303.617209},
  {63.600762, 56.398708, 341.533929, 250.293748, 316.544600, 245.859346, 303.265440},
  {63.602765, 56.396705, 340.944102, 251.498217, 316.135481, 246.927351, 302.902673},
  {63.602924, 56.396546, 340.354250, 252.698414, 315.713982, 247.991562, 302.528932},
  {63.601259, 56.398211, 339.764670, 253.894177, 315.280164, 249.051901, 302.144245},
  {63.597794, 56.401677, 339.175657, 255.085350, 314.834094, 250.108293, 301.748648},
  {63.592557, 56.406916, 338.587502, 256.271780, 314.375848, 251.160662, 301.342182},
  {63.585580, 56.413895, 338.000489, 257.453319, 313.905511, 252.208937, 300.924892},
  {63.576899, 56.422578, 337.414897, 258.629822, 313.423174, 253.253048, 300.496832},
  {63.566555, 56.432926, 336.830999, 259.801150, 312.928934, 254.292926, 300.058058},
  {63.554591, 56.444893, 336.249060, 260.967170, 312.422899, 255.328506, 299.608631},
  {63.541054, 56.458434, 335.669337, 262.127753, 311.905179, 256.359724, 299.148618},
  {63.525993, 56.473499, 335.092080, 263.282775, 311.375893, 257.386519, 298.678092},
  {63.509462, 56.490035, 334.517530, 264.432120, 310.835165, 258.408831, 298.197127},
  {63.491514, 56.507988, 333.945918, 265.575676, 310.283123, 259.426605, 297.705805},
  {63.472207, 56.527301, 333.377467, 266.713336, 309.719903, 260.439787, 297.204210},
  {63.451599, 56.547914, 332.812391, 267.845003, 309.145644, 261.448325, 296.692431},
  {63.429751, 56.569769, 332.250891, 268.970580, 308.560488, 262.452170, 296.170559},
  {63.406723, 56.592803, 331.693162, 270.089983, 307.964582, 263.451277, 295.638691},
  {63.382577, 56.616956, 331.139387, 271.203128, 307.358077, 264.445602, 295.096926},
  {63.357376, 56.642163, 330.589737, 272.309941, 306.741128, 265.435104, 294.545366},
  {63.331184, 56.668363, 330.044375, 273.410354, 306.113889, 266.419746, 293.984115},
  {61.685386, 58.236954, 329.762117, 274.477640, 305.495072, 267.427362, 293.399884},
  {61.751767, 58.248108, 329.475338, 275.543105, 304.870494, 268.432376, 292.810746},
  {61.932321, 58.067527, 329.159000, 276.608374, 304.238477, 269.431171, 292.218171},
  {62.118672, 57.881144, 328.812153, 277.673163, 303.598385, 270.423325, 291.621747},
  {62.311415, 57.688367, 328.433751, 278.737171, 302.949562, 271.408394, 291.021066},
  {62.511166, 57.488576, 328.022646, 279.800081, 302.291322, 272.385905, 290.415718},
  {62.718566, 57.281132, 327.577585, 280.861555, 301.622951, 273.355356, 289.805296},
  {62.934279, 57.065369, 327.097206, 281.921230, 300.943703, 274.316215, 289.189395},
  {63.158992, 56.840601, 326.580037, 282.978716, 300.252799, 275.267914, 288.567615},
  {63.393408, 56.606121, 326.024488, 284.033590, 299.549428, 276.209851, 287.939557},
  {63.638249, 56.361210, 325.428852, 285.085393, 298.832741, 277.141385, 287.304830},
  {63.894242, 56.105139, 324.791302, 286.133624, 298.101850, 278.061835, 286.663047},
  {64.162114, 55.837179, 324.109892, 287.177734, 297.355830, 278.970479, 286.013830},
  {64.442581, 55.556614, 323.382561, 288.217120, 296.593718, 279.866551, 285.356814},
  {64.736328, 55.262757, 322.607133, 289.251116, 295.814507, 280.749239, 284.691644},
  {65.043993, 54.954968, 321.781328, 290.278988, 295.017157, 281.617685, 284.017982},
  {65.366139, 54.632685, 320.902774, 291.299921, 294.200585, 282.470985, 283.335508},
  {65.703223, 54.295450, 319.969025, 292.313012, 293.363679, 283.308189, 282.643924},
  {66.055552, 53.942951, 318.977583, 293.317260, 292.505297, 284.128300, 281.942957},
  {66.423242, 53.575073, 317.925931, 294.311551, 291.624272, 284.930280, 281.232363},
  {66.806156, 53.191953, 316.811576, 295.294649, 290.719428, 285.713052, 280.511932},
  {67.203840, 52.794041, 315.632097, 296.265185, 289.789588, 286.475503, 279.781485},
  {67.615449, 52.382183, 314.385212, 297.221641, 288.833590, 287.216495, 279.040885},
  {68.039665, 51.957696, 313.068856, 298.162342, 287.850310, 287.934872, 278.290035},
  {68.474613, 51.522455, 311.681269, 299.085446, 286.838684, 288.629468, 277.528877},
  {68.917772, 51.078980, 310.221105, 299.988938, 285.797740, 289.299123, 276.757394},
  {69.365904, 50.630514, 308.687546, 300.870627, 284.726629, 289.942699, 275.975607},
  {69.814986, 50.181080, 307.080437, 301.728146, 283.624671, 290.559088, 275.183572},
  {70.260181, 49.735519, 305.400410, 302.558969, 282.491387, 291.147238, 274.381369},
  {70.695847, 49.299480, 303.649025, 303.360424, 281.326556, 291.706160, 273.569100},
  {71.115598, 48.879356, 301.828888, 304.129727, 280.130252, 292.234950, 272.746872},
  {71.512434, 48.482153, 299.943749, 304.864019, 278.902886, 292.732797, 271.914793},
  {71.878949, 48.115288, 297.998573, 305.560426, 277.645249, 293.198996, 271.072955},
  {72.207610, 47.786303, 295.999561, 306.216114, 276.358529, 293.632951, 270.221430},
  {72.491102, 47.502525, 293.954109, 306.828371, 275.044328, 294.034178, 269.360261},
  {72.722720, 47.270668, 291.870715, 307.394678, 273.704651, 294.402303, 268.489462},
  {72.896768, 47.096438, 289.758810, 307.912791, 272.341884, 294.737054, 267.609024},
  {73.008929, 46.984158, 287.628531, 308.380812, 270.958743, 295.038252, 266.718920},
  {73.056569, 46.936467, 285.490448, 308.797254, 269.558214, 295.305804, 265.819126},
  {73.038922, 46.954133, 283.355255, 309.161085, 268.143471, 295.539693, 264.909636},
  {72.957146, 47.035996, 281.233459, 309.471755, 266.717784, 295.739973, 263.990484},
  {72.814233, 47.179059, 279.135074, 309.729199, 265.284435, 295.906764, 263.061761},
  {72.614793, 47.378707, 277.069360, 309.933821, 263.846627, 296.040250, 262.123636},
  {72.323854, 47.601984, 275.045742, 310.086386, 262.408218, 296.140639, 261.176904},
  {71.709262, 47.681090, 273.078913, 310.187898, 260.977630, 296.208107, 260.225672},
  {70.585776, 47.469217, 271.186711, 310.240343, 259.566889, 296.243345, 259.276941},
  {68.971142, 46.948578, 269.384065, 310.247118, 258.187483, 296.247927, 258.337981},
  {66.883392, 46.101103, 267.682949, 310.212791, 256.850255, 296.224237, 257.416263},
  {64.379475, 44.937727, 266.091570, 310.142788, 255.564570, 296.175348, 256.518840},
  {61.707481, 43.610302, 264.610254, 310.042678, 254.334488, 296.104575, 255.649510},
  {59.067894, 42.250386, 263.233689, 309.917523, 253.159778, 296.015033, 254.809259},
  {56.462460, 40.856241, 261.956276, 309.771921, 252.039955, 295.909659, 253.998958},
  {53.891644, 39.427412, 260.772342, 309.610029, 250.974350, 295.791204, 253.219357},
  {51.354969, 37.964386, 259.676291, 309.435591, 249.962172, 295.662237, 252.471102},
  {48.851295, 36.468309, 258.662716, 309.251964, 249.002557, 295.525147, 251.754736},
  {46.379041, 34.940771, 257.726471, 309.062159, 248.094603, 295.382144, 251.070708},
  {43.936351, 33.383633, 256.862711, 308.868865, 247.237399, 295.235269, 250.419391},
  {41.521220, 31.798907, 256.066922, 308.674487, 246.430044, 295.086400, 249.801082},
  {39.131585, 30.188661, 255.334929, 308.481171, 245.671663, 294.937259, 249.216018},
  {36.765389, 28.554954, 254.662892, 308.290830, 244.961418, 294.789422, 248.664386},
  {34.420623, 26.899798, 254.047301, 308.105170, 244.298514, 294.644325, 248.146326},
  {32.095357, 25.225129, 253.484963, 307.925710, 243.682203, 294.503277, 247.661941},
  {29.787751, 23.532788, 252.972986, 307.753798, 243.111791, 294.367463, 247.211307},
  {27.496065, 21.824517, 252.508762, 307.590634, 242.586633, 294.237953, 246.794472},
  {25.218662, 20.101953, 252.089953, 307.437278, 242.106139, 294.115711, 246.411468},
  {22.954005, 18.366638, 251.714471, 307.294666, 241.669772, 294.001599, 246.062309},
  {20.700650, 16.620014, 251.380466, 307.163620, 241.277046, 293.896386, 245.747000},
  {18.457242, 14.863438, 251.086309, 307.044859, 240.927526, 293.800748, 245.465535},
  {16.222506, 13.098187, 250.830580, 306.939005, 240.620830, 293.715281, 245.217906},
  {13.995238, 11.325463, 250.612055, 306.846590, 240.356621, 293.640495, 245.004099},
  {11.774298, 9.546409, 250.429700, 306.768064, 240.134614, 293.576827, 244.824099},
  {9.558599, 7.762113, 250.282656, 306.703797, 239.954568, 293.524638, 244.677892},
  {7.347099, 5.973615, 250.170234, 306.654086, 239.816290, 293.484220, 244.565464},
  {5.176238, 4.212406, 250.091344, 306.618900, 239.718928, 293.455587, 244.486231},
  {3.265459, 2.659042, 250.041708, 306.596634, 239.657532, 293.437456, 244.436236},
  {1.796746, 1.463612, 250.014440, 306.584361, 239.623758, 293.427458, 244.408724},
  {0.769374, 0.626841, 250.002774, 306.579100, 239.609298, 293.423171, 244.396943},
  {0.182980, 0.149092, 250.000000, 306.577848, 239.605859, 293.422152, 244.394141},
  {0.000000, 0.000000, 250.000000, 306.577848, 239.605859, 293.422152, 244.394141}};
}}