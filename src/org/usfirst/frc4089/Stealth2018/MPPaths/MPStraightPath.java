package org.usfirst.frc4089.Stealth2018.MPPaths;			
public class MPStraightPath {			
	public static final int kNumPoints =310;		
	// Position (rotations)	Velocity (RPM)	Duration (ms)
	public static double [][]Points = new double[][]{		
	{0,	0	,10},
	{4.76190476190476E-05,	0.571428571	,10},
	{0.000214285714285714,	1.428571429	,10},
	{0.000547619047619048,	2.571428571	,10},
	{0.0010952380952381,	4	,10},
	{0.0019047619047619,	5.714285714	,10},
	{0.00302380952380952,	7.714285714	,10},
	{0.0045,	10	,10},
	{0.00638095238095238,	12.57142857	,10},
	{0.00871428571428571,	15.42857143	,10},
	{0.011547619047619,	18.57142857	,10},
	{0.0149285714285714,	22	,10},
	{0.0189047619047619,	25.71428571	,10},
	{0.0235238095238095,	29.71428571	,10},
	{0.0288333333333333,	34	,10},
	{0.0348809523809524,	38.57142857	,10},
	{0.0417142857142857,	43.42857143	,10},
	{0.0493809523809524,	48.57142857	,10},
	{0.0579285714285714,	54	,10},
	{0.0674047619047619,	59.71428571	,10},
	{0.0778571428571429,	65.71428571	,10},
	{0.0893095238095238,	71.71428571	,10},
	{0.101761904761905,	77.71428571	,10},
	{0.115214285714286,	83.71428571	,10},
	{0.129666666666667,	89.71428571	,10},
	{0.145119047619048,	95.71428571	,10},
	{0.161571428571429,	101.7142857	,10},
	{0.17902380952381,	107.7142857	,10},
	{0.19747619047619,	113.7142857	,10},
	{0.216928571428571,	119.7142857	,10},
	{0.237380952380952,	125.7142857	,10},
	{0.258833333333333,	131.7142857	,10},
	{0.281285714285714,	137.7142857	,10},
	{0.304738095238095,	143.7142857	,10},
	{0.329190476190476,	149.7142857	,10},
	{0.354642857142857,	155.7142857	,10},
	{0.381095238095238,	161.7142857	,10},
	{0.408547619047619,	167.7142857	,10},
	{0.437,	173.7142857	,10},
	{0.466452380952381,	179.7142857	,10},
	{0.496904761904762,	185.7142857	,10},
	{0.528309523809524,	191.1428571	,10},
	{0.560595238095238,	196.2857143	,10},
	{0.593714285714286,	201.1428571	,10},
	{0.627619047619048,	205.7142857	,10},
	{0.662261904761905,	210	,10},
	{0.697595238095238,	214	,10},
	{0.733571428571429,	217.7142857	,10},
	{0.770142857142857,	221.1428571	,10},
	{0.807261904761905,	224.2857143	,10},
	{0.844880952380952,	227.1428571	,10},
	{0.882952380952381,	229.7142857	,10},
	{0.921428571428571,	232	,10},
	{0.960261904761905,	234	,10},
	{0.999404761904762,	235.7142857	,10},
	{1.03880952380952,	237.1428571	,10},
	{1.07842857142857,	238.2857143	,10},
	{1.11821428571429,	239.1428571	,10},
	{1.15811904761905,	239.7142857	,10},
	{1.19809523809524,	240	,10},
	{1.23809523809524,	240	,10},
	{1.27809523809524,	240	,10},
	{1.31809523809524,	240	,10},
	{1.35809523809524,	240	,10},
	{1.39809523809524,	240	,10},
	{1.43809523809524,	240	,10},
	{1.47809523809524,	240	,10},
	{1.51809523809524,	240	,10},
	{1.55809523809524,	240	,10},
	{1.59809523809524,	240	,10},
	{1.63809523809524,	240	,10},
	{1.67809523809524,	240	,10},
	{1.71809523809524,	240	,10},
	{1.75809523809524,	240	,10},
	{1.79809523809524,	240	,10},
	{1.83809523809524,	240	,10},
	{1.87809523809524,	240	,10},
	{1.91809523809524,	240	,10},
	{1.95809523809524,	240	,10},
	{1.99809523809524,	240	,10},
	{2.03809523809524,	240	,10},
	{2.07809523809524,	240	,10},
	{2.11809523809524,	240	,10},
	{2.15809523809524,	240	,10},
	{2.19809523809524,	240	,10},
	{2.23809523809524,	240	,10},
	{2.27809523809524,	240	,10},
	{2.31809523809524,	240	,10},
	{2.35809523809524,	240	,10},
	{2.39809523809524,	240	,10},
	{2.43809523809524,	240	,10},
	{2.47809523809524,	240	,10},
	{2.51809523809524,	240	,10},
	{2.55809523809524,	240	,10},
	{2.59809523809524,	240	,10},
	{2.63809523809524,	240	,10},
	{2.67809523809524,	240	,10},
	{2.71809523809524,	240	,10},
	{2.75809523809524,	240	,10},
	{2.79809523809524,	240	,10},
	{2.83809523809524,	240	,10},
	{2.87809523809524,	240	,10},
	{2.91809523809524,	240	,10},
	{2.95809523809524,	240	,10},
	{2.99809523809524,	240	,10},
	{3.03809523809524,	240	,10},
	{3.07809523809524,	240	,10},
	{3.11809523809524,	240	,10},
	{3.15809523809524,	240	,10},
	{3.19809523809524,	240	,10},
	{3.23809523809524,	240	,10},
	{3.27809523809524,	240	,10},
	{3.31809523809524,	240	,10},
	{3.35809523809524,	240	,10},
	{3.39809523809524,	240	,10},
	{3.43809523809524,	240	,10},
	{3.47809523809524,	240	,10},
	{3.51809523809524,	240	,10},
	{3.55809523809524,	240	,10},
	{3.59809523809524,	240	,10},
	{3.63809523809524,	240	,10},
	{3.67809523809524,	240	,10},
	{3.71809523809524,	240	,10},
	{3.75809523809524,	240	,10},
	{3.79809523809524,	240	,10},
	{3.83809523809524,	240	,10},
	{3.87809523809524,	240	,10},
	{3.91809523809524,	240	,10},
	{3.95809523809524,	240	,10},
	{3.99809523809524,	240	,10},
	{4.03809523809524,	240	,10},
	{4.07809523809524,	240	,10},
	{4.11809523809524,	240	,10},
	{4.15809523809524,	240	,10},
	{4.19809523809524,	240	,10},
	{4.23809523809524,	240	,10},
	{4.27809523809524,	240	,10},
	{4.31809523809524,	240	,10},
	{4.35809523809524,	240	,10},
	{4.39809523809524,	240	,10},
	{4.43809523809524,	240	,10},
	{4.47809523809524,	240	,10},
	{4.51809523809524,	240	,10},
	{4.55809523809524,	240	,10},
	{4.59809523809524,	240	,10},
	{4.63809523809524,	240	,10},
	{4.67809523809524,	240	,10},
	{4.71809523809524,	240	,10},
	{4.75809523809524,	240	,10},
	{4.79809523809524,	240	,10},
	{4.83809523809524,	240	,10},
	{4.87809523809524,	240	,10},
	{4.91809523809524,	240	,10},
	{4.95809523809524,	240	,10},
	{4.99809523809524,	240	,10},
	{5.03809523809524,	240	,10},
	{5.07809523809524,	240	,10},
	{5.11809523809524,	240	,10},
	{5.15809523809524,	240	,10},
	{5.19809523809524,	240	,10},
	{5.23809523809524,	240	,10},
	{5.27809523809524,	240	,10},
	{5.31809523809524,	240	,10},
	{5.35809523809524,	240	,10},
	{5.39809523809524,	240	,10},
	{5.43809523809524,	240	,10},
	{5.47809523809524,	240	,10},
	{5.51809523809524,	240	,10},
	{5.55809523809524,	240	,10},
	{5.59809523809524,	240	,10},
	{5.63809523809524,	240	,10},
	{5.67809523809524,	240	,10},
	{5.71809523809524,	240	,10},
	{5.75809523809524,	240	,10},
	{5.79809523809524,	240	,10},
	{5.83809523809524,	240	,10},
	{5.87809523809524,	240	,10},
	{5.91809523809524,	240	,10},
	{5.95809523809524,	240	,10},
	{5.99809523809524,	240	,10},
	{6.03809523809524,	240	,10},
	{6.07809523809524,	240	,10},
	{6.11809523809524,	240	,10},
	{6.15809523809524,	240	,10},
	{6.19809523809524,	240	,10},
	{6.23809523809524,	240	,10},
	{6.27809523809524,	240	,10},
	{6.31809523809524,	240	,10},
	{6.35809523809524,	240	,10},
	{6.39809523809524,	240	,10},
	{6.43809523809524,	240	,10},
	{6.47809523809524,	240	,10},
	{6.51809523809524,	240	,10},
	{6.55809523809524,	240	,10},
	{6.59809523809524,	240	,10},
	{6.63809523809524,	240	,10},
	{6.67809523809524,	240	,10},
	{6.71809523809524,	240	,10},
	{6.75809523809524,	240	,10},
	{6.79809523809524,	240	,10},
	{6.83809523809524,	240	,10},
	{6.87809523809524,	240	,10},
	{6.91809523809524,	240	,10},
	{6.95809523809524,	240	,10},
	{6.99809523809524,	240	,10},
	{7.03809523809524,	240	,10},
	{7.07809523809524,	240	,10},
	{7.11809523809524,	240	,10},
	{7.15809523809524,	240	,10},
	{7.19809523809524,	240	,10},
	{7.23809523809524,	240	,10},
	{7.27809523809524,	240	,10},
	{7.31809523809524,	240	,10},
	{7.35809523809524,	240	,10},
	{7.39809523809524,	240	,10},
	{7.43809523809524,	240	,10},
	{7.47809523809524,	240	,10},
	{7.51809523809524,	240	,10},
	{7.55809523809524,	240	,10},
	{7.59809523809524,	240	,10},
	{7.63809523809524,	240	,10},
	{7.67809523809524,	240	,10},
	{7.71809523809524,	240	,10},
	{7.75809523809524,	240	,10},
	{7.79809523809524,	240	,10},
	{7.83809523809524,	240	,10},
	{7.87809523809524,	240	,10},
	{7.91809523809524,	240	,10},
	{7.95809523809524,	240	,10},
	{7.99809523809524,	240	,10},
	{8.03809523809524,	240	,10},
	{8.07809523809524,	240	,10},
	{8.11809523809524,	240	,10},
	{8.15809523809524,	240	,10},
	{8.19809523809524,	240	,10},
	{8.23809523809524,	240	,10},
	{8.27809523809524,	240	,10},
	{8.31809523809524,	240	,10},
	{8.35809523809524,	240	,10},
	{8.39809523809524,	240	,10},
	{8.43809523809523,	240	,10},
	{8.47809523809523,	240	,10},
	{8.51809523809523,	240	,10},
	{8.55809523809523,	240	,10},
	{8.59809523809523,	240	,10},
	{8.63809523809523,	240	,10},
	{8.67809523809523,	240	,10},
	{8.71809523809523,	240	,10},
	{8.75809523809523,	240	,10},
	{8.79809523809523,	240	,10},
	{8.83809523809523,	240	,10},
	{8.87804761904761,	239.4285714	,10},
	{8.91788095238094,	238.5714286	,10},
	{8.95754761904761,	237.4285714	,10},
	{8.99699999999999,	236	,10},
	{9.03619047619047,	234.2857143	,10},
	{9.07507142857142,	232.2857143	,10},
	{9.11359523809523,	230	,10},
	{9.15171428571428,	227.4285714	,10},
	{9.18938095238094,	224.5714286	,10},
	{9.22654761904761,	221.4285714	,10},
	{9.26316666666666,	218	,10},
	{9.29919047619047,	214.2857143	,10},
	{9.33457142857142,	210.2857143	,10},
	{9.3692619047619,	206	,10},
	{9.40321428571428,	201.4285714	,10},
	{9.43638095238094,	196.5714286	,10},
	{9.46871428571428,	191.4285714	,10},
	{9.50016666666666,	186	,10},
	{9.53069047619047,	180.2857143	,10},
	{9.56023809523809,	174.2857143	,10},
	{9.5887857142857,	168.2857143	,10},
	{9.61633333333332,	162.2857143	,10},
	{9.64288095238094,	156.2857143	,10},
	{9.66842857142856,	150.2857143	,10},
	{9.69297619047618,	144.2857143	,10},
	{9.7165238095238,	138.2857143	,10},
	{9.73907142857142,	132.2857143	,10},
	{9.76061904761904,	126.2857143	,10},
	{9.78116666666666,	120.2857143	,10},
	{9.80071428571428,	114.2857143	,10},
	{9.81926190476189,	108.2857143	,10},
	{9.83680952380951,	102.2857143	,10},
	{9.85335714285713,	96.28571429	,10},
	{9.86890476190475,	90.28571429	,10},
	{9.88345238095237,	84.28571429	,10},
	{9.89699999999999,	78.28571429	,10},
	{9.90954761904761,	72.28571429	,10},
	{9.92109523809523,	66.28571429	,10},
	{9.93164285714285,	60.28571429	,10},
	{9.94119047619047,	54.28571429	,10},
	{9.9497857142857,	48.85714286	,10},
	{9.95749999999999,	43.71428571	,10},
	{9.96438095238094,	38.85714286	,10},
	{9.97047619047618,	34.28571429	,10},
	{9.97583333333332,	30	,10},
	{9.98049999999999,	26	,10},
	{9.9845238095238,	22.28571429	,10},
	{9.98795238095237,	18.85714286	,10},
	{9.99083333333332,	15.71428571	,10},
	{9.99321428571427,	12.85714286	,10},
	{9.99514285714285,	10.28571429	,10},
	{9.99666666666666,	8	,10},
	{9.99783333333332,	6	,10},
	{9.99869047619046,	4.285714286	,10},
	{9.9992857142857,	2.857142857	,10},
	{9.99966666666666,	1.714285714	,10},
	{9.99988095238094,	0.857142857	,10},
	{9.99997619047618,	0.285714286	,10},
	{9.99999999999999,	0	,10}};
}