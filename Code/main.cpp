#include <iostream> 
#include <list> 
#include <stack> 
#include <queue> 
#include <algorithm>

/*#include <math.h>
#define _USE_MATH_DEFINES*/

using namespace std;

#define M 3 
#define N 10 
#define NO 0 
#define YES 5 
#define BFSAdd 4
const float PI = 3.141592654;

typedef enum {
	WHITE = 0,
	GRAY,
	BLACK
} NodeMarker;


// Breadth First Search
typedef struct Node {
	int row;
	int col;
} Node;


class Mat {

public:
	int value;
	NodeMarker visited;
	int parentRow;
	int parentCol;
};


typedef struct Location {
	int x;
	int y;
} Location;


class Point2D {
public:
	int x;
	int y;
};


typedef struct ConvexConcaveIndices
{
	vector<int> ConvexIndices;
	vector<int> ConcaveIndices;
} ConvexConcaveIndices;

// Declering classes in order to enable them to refer to each other. 
class Triangule;
class Polygon;
class LineSegment;


class Polygon {
private:

public:
	vector<Point2D> points2D;

	Polygon() {}
	Polygon(vector<Point2D> vect) {
		points2D = vect;
	}

	float SumOfInteriorAngles(int DegRad = 1); // AnglesRadians: 1 for Degrees, 0 for radians
	float SumOfExteriosAngles(int DegRad = 1); // always equals 360 (for any n sides polygon)
	float SumOfInteriorComplementaryAngles(int DegRad = 1);
	bool IsClockWise(int Method = 3);
	vector<Point2D> GetPolygonClockWiseOrder(int Method = 3);
	vector<Point2D> GetPolygonCounterClockWiseOrder(int Method = 3);
	int getBottomVertexIndex();
	Point2D getBottomVertex();
	int GetRightmostVertexIndex(int pointindex);
	vector<float> CalcPolygonAngles(int DegRad = 1); // with correspondence to polygon's 2D points vector
	vector<bool> ConvexVertices(); // with correspondence to polygon's 2D points vector
	vector<bool> ConcaveVertices(); // concave vertex = reflexvertex. with correspondence to polygon's 2D points vector
	bool IsConvexVertex(int index); // with correspondence topolygon's 2D points vector
	bool IsConcaveVertex(int index); // concave vertex = reflexvertex. with correspondence to polygon's 2D points vector
	bool IsConvexVertex(float angle, int DegRad = 1);
	bool IsConcaveVertex(float angle, int DegRad = 1);
	bool IsConvexPolygon();
	bool IsConcavePolygon();
	int NumberOfConvexVertices();
	int NumberOfConcaveVertices();
	ConvexConcaveIndices GetConvexConcaveIndices();
	Polygon GetSubPolygon(vector<int> points2DIndices);
	Polygon* translatePolygon(Point2D move);
	vector<Point2D> GetPolygonVertices(); // returns a copy 
	vector<Point2D> GetReversePolygonVertices(); // returns a copy 
	vector<Triangule> PolygonTriangulation();

};


class Triangule : public Polygon {
private:

public:
	Triangule(vector<Point2D> vect) { // Takes first three 2D points of vect.

		points2D = { vect[0], vect[1], vect[2] };
		//points2D[0] = vect[0]; 
		//points2D[1] = vect[1]; 
		//points2D[2] = vect[2];
	}

};

class LineSegment : public Polygon {
private:

public:
	LineSegment(vector<Point2D> vect) { // Takes first 2 2D points of vect.

		points2D = { vect[0], vect[1] };
		//points2D[0] = vect[0]; 
		//points2D[1] = vect[1]; 
		//points2D[2] = vect[2];
	}

};


typedef struct BBOX
{
	int a0;
	int a1;
	int b0;
	int b1;
} BBOX;


typedef struct LineFormula {
	float A;
	float B;
	float C;
} LineFormula;


void printMatValue(Mat mat[][N]);
void printMatVisited(Mat mat[][N]);
void printMatParent(Mat mat[][N]);
void printAnyMat(vector<vector<int>> mat);
void printAnyMatXY(vector<vector<int>> mat);
void SetValue(Mat mat[][N], int rowstart, int rowEnd, int colStart, int colEnd, int value);
void SetVisited(Mat mat[][N], int rowstart, int rowEnd, int colstart, int colEnd, NodeMarker visited);
void SetParent(Mat mat[][N], int rowstart, int rowEnd, int colStart, int colEnd, int parentRow, int parentCol);
void BFS(Mat mat[][N], int SourceRow, int SourceCol);
void printBFS(Mat matorigional[][N], int sourceRow, int sourceCol, int destRow, int destCol);
Location NextFirstUnassignedLocation(int a[][9], int x, int y);
bool isLegal(int a[][9], int x, int y, int num);
bool SolveSoduku(int a[][9], int x, int y, bool print);
void printMatSuduku(int a[][9]);
void drawCircle(int a, int b, int r, vector<vector<int>> & mat, int value = 1);
void drawLine(int x1, int x2, int y1, int y2, vector<vector<int>> & mat, int value = 1);
float CosAlphaSquared(int xl, int x2, int y1, int y2);
float SinAlphaSquared(int xl, int x2, int yl, int y2);
void drawPolygon(Polygon & poly, vector<vector<int>> & mat, int value = 1);
void drawPolygon(Polygon & poly, vector<vector<int>> & mat, vector<float> value);
void drawBBOX(BBOX bbox, vector<vector<int>> & m, int value = 1);
BBOX getBBOX(Polygon poly);
int InOutPolygon(int x, int y, Polygon poly);
float d(float xl, float yl, LineFormula lineEq);
LineFormula getLineFormula(float xl, float yl, float x2, float y2);
Point2D arbitraryPointInsideTriangule(Triangule triangule);
void drawPoint(Point2D p, vector<vector<int>> & mat, int value = 1);
bool SameSign(float a, float b);
int InOutTriangule(Triangule triangule, Point2D pQuery);
void TestInOutTriangule(Triangule triangule, vector<vector<int>> mat);
bool isOnLineSegment(Point2D pl, Point2D p2, Point2D pQuery);
void TestIsOnLineSegment(LineSegment lineseg, vector<vector<int>> mat);
void drawClockWisePolygon(Polygon poly, vector<vector<int>> & mat, int Method = 3);
void drawCounterClockWisePolygon(Polygon poly, vector<vector<int>> & mat, int Method = 3);
float CalcAngleBetweenLineSegments(LineSegment linesegmentl, LineSegment linesegment2, int DegRad = 1);
void TestGetSubPolygon();
void printVector(vector<int>);

// page 3


void printIntList(list<int> mylntList);
list<int> vectorIntToIntList(vector<int> intVector);
vector<int> listIntToVectorList(list<int> intList);
vector<Point2D> translate(vector<Point2D> vectPoints2D, Point2D move);
vector<Point2D> scale(vector<Point2D> vectPoints2D, float scalar);
void TestPolygonTriangulation();
void drawMultipleTriangules(vector<Triangule> triangulesvector, vector<vector<int>> & mat);
void drawMultiplePolygons(vector<Polygon> polygonsvector, vector<vector<int>> & mat);


int main()
{
	cout << "start" << endl;

	cout << (-1) % 7 << endl;
	cout << (-1 + 7) % 7 << endl;
	cout << (0 + 7) % 7 << endl;
	cout << (1 + 7) % 7 << endl;
	cout << (2 + 7) % 7 << endl;
	cout << (3 + 7) % 7 << endl;
	cout << (4 + 7) % 7 << endl;
	cout << (5 + 7) % 7 << endl;
	cout << (6 + 7) % 7 << endl;
	cout << (7 + 7) % 7 << endl;

	vector<vector<int>> m(30, vector<int>(30));

	//// out « "Draw Line" « endl; 
	////printAnymat(m);
	//drawLine(7, 10, 5, 5, m); 
	//printAnymat(m);

	////printAnymat(m);
	//drawLine(16, 20, 15, 16, m); 
	//printAnymat(m);

	////printAnymat(m);
	//drawLine(18, 18, 5, 10, m); 
	//printAnyMat(m);

	////printAnyMat(m);
	//drawLine(6, 10, 15, 21, m); 
	//printAnymat(m);

	////printAnymat(m);
	//drawLine(3, 0, 5, 5, m); 
	//printAnyMat(m);

	////printAnymat(m);
	//drawLine(29, 29, 18, 14, m); 
	//printAnyMat(m);

	////printAnymat(m);
	//drawLine(24, 21, 14, 18, m); 
	//printAnymat(m);

	//cout « "Draw Circle" « endl; 
	////vector<vector<int» m(20, vector<int>(20)); 
	////printAnymat(m);
	//drawcircle(10,10,5,m);
	//printAnyMat(m);

	cout << "Draw Polygon" << endl;
	//vector<Point2D> vect2D = { { 3, 4 }, { 4, 26 },  { 13, 24 }, { 27, 28}, { 17, 11 }, { 17, 2 }, { 10, 3 } };
	vector<Point2D> vect2D = { { 3, 5 },{ 4, 26 },{ 13, 24 },{ 27, 28 },{ 17, 11 },{ 17, 2 },{ 10, 3 } };
	Polygon nonConvexPolyExample(vect2D);
	drawPolygon(nonConvexPolyExample, m);
	printAnyMat(m);

	cout << "Draw Polygon BBOX" << endl;
	drawBBOX(getBBOX(nonConvexPolyExample), m);
	printAnyMat(m);


	int Line1_x1 = 1;
	int Line1_y1 = 13;
	int Line1_x2 = 14;
	int Line1_y2 = 7;
	drawLine(Line1_x1, Line1_x2, Line1_y1, Line1_y2, m);
	printAnyMat(m);


	LineFormula lineEQ = getLineFormula(Line1_x1, Line1_y1, Line1_x2, Line1_y2);
	vector<float> disVect(nonConvexPolyExample.points2D.size());
	int i = 0;

	// for each (Point2D p2D in nonConvexPolyExample.points2D)
	for (Point2D p2D : nonConvexPolyExample.points2D)
	{
		disVect[i] = d(p2D.x, p2D.y, lineEQ);
		i++;
	}

	vector<vector<int>> m2(30, vector<int>(30));

	Triangule triangule1(nonConvexPolyExample.points2D);
	Triangule triangule2({ nonConvexPolyExample.points2D[3],nonConvexPolyExample.points2D[4], nonConvexPolyExample.points2D[5] });


	drawPolygon(triangule1, m2);
	drawPolygon(triangule2, m2);
	printAnyMat(m2);

	Point2D p1 = arbitraryPointInsideTriangule(triangule1);
	Point2D p2 = arbitraryPointInsideTriangule(triangule2);
	drawPoint(p1, m2);
	drawPoint(p2, m2);
	printAnyMat(m2);

	TestInOutTriangule(triangule1, m2);
	TestInOutTriangule(triangule2, m2);

	TestIsOnLineSegment(LineSegment({ triangule1.points2D[0], triangule1.points2D[1] }), m2);


	cout << endl << "Deg Interior Angles Sum: " << nonConvexPolyExample.SumOfInteriorAngles() << endl;
	cout << endl << "Rad Interior Angles sum: " << nonConvexPolyExample.SumOfInteriorAngles(0) << endl;
	cout << endl << "Deg Exterior Angles Sum: " << nonConvexPolyExample.SumOfExteriosAngles() << endl;
	cout << endl << "Rad Exterior Angles Sum: " << nonConvexPolyExample.SumOfExteriosAngles(0) << endl;
	cout << endl << "Deg Interior Complementary Angles Sum: " << nonConvexPolyExample.SumOfInteriorComplementaryAngles() << endl;
	cout << endl << "Rad Interior complementary Angles sum: " << nonConvexPolyExample.SumOfInteriorComplementaryAngles(0) << endl;

	// clock wise testing
	for (int mth = 1; mth <= 3; mth++)
	{
		vector<vector<int>> m3(30, vector<int>(30));
		drawClockWisePolygon(nonConvexPolyExample, m3, mth);


		//Page 5

		printAnyMatXY(m3);

		vector<vector<int>> m4(30, vector<int>(30));
		reverse(nonConvexPolyExample.points2D.begin(), nonConvexPolyExample.points2D.end());
		drawClockWisePolygon(nonConvexPolyExample, m4, mth);
		printAnyMatXY(m4);

		vector<vector<int>> m5(30, vector<int>(30));
		reverse(nonConvexPolyExample.points2D.begin(), nonConvexPolyExample.points2D.end());
		drawClockWisePolygon(nonConvexPolyExample, m5, mth);
		printAnyMatXY(m5);

		// counter clock wise testing
		vector<vector<int>> m6(30, vector<int>(30));
		drawCounterClockWisePolygon(nonConvexPolyExample, m6, mth);
		printAnyMatXY(m6);

		vector<vector<int>> m7(30, vector<int>(30));
		reverse(nonConvexPolyExample.points2D.begin(), nonConvexPolyExample.points2D.end());
		drawCounterClockWisePolygon(nonConvexPolyExample, m7, mth);
		printAnyMatXY(m7);

		vector<vector<int>> m8(30, vector<int>(30));
		reverse(nonConvexPolyExample.points2D.begin(), nonConvexPolyExample.points2D.end());
		drawCounterClockWisePolygon(nonConvexPolyExample, m8, mth);
		printAnyMatXY(m8);
	}

	vector<vector<int>> m9(30, vector<int>(30));
	drawPolygon(nonConvexPolyExample, m9);
	drawPoint(nonConvexPolyExample.getBottomVertex(), m9, 10);
	printAnyMatXY(m9);
	drawPolygon(triangule1, m9);
	drawPoint(triangule1.getBottomVertex(), m9, 10);
	drawPolygon(triangule2, m9);
	drawPoint(triangule2.getBottomVertex(), m9, 10);
	printAnyMatXY(m9);

	vector<vector<int>> m10(30, vector<int>(30));
	drawPolygon(nonConvexPolyExample, m10, { 1, 2, 3 });
	printAnyMatXY(m10);
	vector<float> polyAngles = nonConvexPolyExample.CalcPolygonAngles();
	drawPolygon(nonConvexPolyExample, m10, polyAngles);
	printAnyMatXY(m10);

	float sumPolyAngles = 0;
	for (int i = 0; i < polyAngles.size(); i++)
	{
		sumPolyAngles += polyAngles[i];
	}

	cout << endl << endl;
	cout << "Sum of Polygon Interior Angles: " << sumPolyAngles << endl;
	cout << "Sum of Interior Angles of any " << nonConvexPolyExample.points2D.size() << " size polygon: " << nonConvexPolyExample.SumOfInteriorAngles() << endl;


	vector<vector<int>> m11(30, vector<int>(30));
	vector<bool> convexVertices = nonConvexPolyExample.ConvexVertices();
	vector<float> convexColors(convexVertices.size());
	for (int i = 0; i < convexColors.size(); i++)
	{
		convexColors[i] = (convexVertices[i] ? 2 : 1); // 2 for convex vertex, 1 for non-convex vertex.


													   //Page 6

	}

	drawPolygon(nonConvexPolyExample, m11, convexColors);
	printAnyMatXY(m11);


	vector<vector<int>> m12(30, vector<int>(30));
	vector<bool> concaveVertices = nonConvexPolyExample.ConcaveVertices();
	vector<float> concaveColors(concaveVertices.size());
	for (int i = 0; i < concaveColors.size(); i++)
	{
		concaveColors[i] = (concaveVertices[i] ? 1 : 2); // 1 for concave vertex, 2 for non-concave vertex.
	}

	drawPolygon(nonConvexPolyExample, m12, concaveColors);
	printAnyMatXY(m12);


	ConvexConcaveIndices convexAndconcaveIndices = nonConvexPolyExample.GetConvexConcaveIndices();
	cout << endl << "convex indices: ";
	for (int i = 0; i < convexAndconcaveIndices.ConvexIndices.size(); i++)
	{
		cout << convexAndconcaveIndices.ConvexIndices[i] << ", ";
	}
	cout << endl;

	cout << "concave indices: ";
	for (int i = 0; i < convexAndconcaveIndices.ConcaveIndices.size(); i++)
	{
		cout << convexAndconcaveIndices.ConcaveIndices[i] << ", ";
	}
	cout << endl;


	TestGetSubPolygon();


	cout << endl;
	vector<Point2D> polyVertices = nonConvexPolyExample.GetPolygonVertices();
	polyVertices[0] = { -1, -1 };
	// should be equal -1 both
	cout << polyVertices[0].x << "," << polyVertices[0].y << " should be equal - 1 both" << endl;

	polyVertices = nonConvexPolyExample.GetPolygonVertices();
	// Both should not be equal -1
	cout << polyVertices[0].x << "," << polyVertices[0].y << " Both should not be equal - 1" << endl;

	vector<Point2D> polyReverseVertices = nonConvexPolyExample.GetReversePolygonVertices(); // Both should not be equal -1
	cout << polyReverseVertices[polyReverseVertices.size() - 1].x << "," << polyReverseVertices[polyReverseVertices.size() - 1].y << " Both should not be equal - 1" << endl;

	// size 5 --> {0,1,2,3,4} --> {0,4,3,2,1}
	vector<int> pointsFlippedPolygonIndices(nonConvexPolyExample.GetPolygonVertices().size());

	for (int i = 0; i < pointsFlippedPolygonIndices.size(); i++)
	{
		pointsFlippedPolygonIndices[(pointsFlippedPolygonIndices.size() - i) % pointsFlippedPolygonIndices.size()] = i;
	}
	for (int i = 0; i < pointsFlippedPolygonIndices.size(); i++)


		//Page 7

	{
		cout << pointsFlippedPolygonIndices[i] << ",";
	}
	cout << endl;
	printVector(pointsFlippedPolygonIndices);


	TestPolygonTriangulation();


	cout << "sorted and Rotated arrays (distinct values)" << endl;

	vector<int> v = { 1, 2, 5, 7, 8, 10, 12, 14, 15, 17, 21, 30, 31, 33, 35, 38 ,40 };
	int x = 7; // value to search

	cout << endl;
	for (int i = 0; i < v.size(); i++)
	{
		cout << v[i] << ",";
	}

	// perform binary search 
	int low = 0;
	int high = v.size() - 1;
	int index = -1;

	while (low <= high && index == -1)
	{
		int mid = (low + high) / 2;

		if (v[mid] == x)
		{
			index = mid;
		}
		else if (x < v[mid])
		{
			high = mid - 1;
		}
		else
		{
			low = mid + 1;
		}
	}

	cout << endl << "{index, v[index]} = {" << index << "," << v[index] << "}" << endl;

	// create rotated arrays and find pivot.

	for (int R = 0; R < v.size(); R++)
	{
		// print Rotated array
		vector<int> vRotated = v;
		for (int i = 0; i < v.size(); i++)
		{
			vRotated[i] = v[(i + v.size() - R) % v.size()];
		}
		cout << endl;
		for (int i = 0; i < vRotated.size(); i++)
		{
			cout << vRotated[i] << ",";
		}

		// perform binary search over rotated array 
		low = 0;
		high = vRotated.size() - 1;


		//Page 8

		int Rindex = -1;

		while (low < high)
		{
			int mid = (low + high) / 2;

			if (vRotated[mid] > vRotated[low])
			{ // All elements from low to mid-1 are fine.
				low = mid;
			}
			else
			{
				high = mid;
				Rindex = high + 1;
			}
		}

		if ((Rindex == vRotated.size() - 1) && (vRotated[vRotated.size() - 1] > vRotated[0]))
		{
			Rindex = 0;
		}


		// perform binary search on arrays
		// 1. find the correct subarray
		// 2. perform the search

		low = 0;
		high = vRotated.size() - 1;
		index = -1;

		if (x >= vRotated[Rindex] && x <= vRotated[vRotated.size() - 1])
			// left subarray
		{
			low = Rindex;
			high = vRotated.size() - 1;
		}
		else
		{
			low = 0;
			high = Rindex - 1;
		}

		while (low <= high && index == -1)
		{
			int mid = (low + high) / 2;

			if (vRotated[mid] == x)
			{
				index = mid;
			}
			else if (x < vRotated[mid])
			{
				high = mid - 1;
			}
			else
			{
				low = mid + 1;
			}
		}

		cout << endl << "R = " << R << ": {Rotatedlndex, vRotated[Rotatedindex]} = { " << Rindex << "," << vRotated[Rindex] << " }" << endl;
		cout << endl << "{index, vRotated[index]} = {" << index << "," << vRotated[index] << "}" << endl;
	}


	// Page 9

	cout << "Suduku" << endl;

	int a[9][9] = { 0, 1, 0, 5, 2, 0, 0, 6, 9,
					7, 0, 3, 4, 0, 0, 0, 5, 2,
					0, 4, 6, 7, 0, 9, 0, 0, 1,
					0 ,0, 0, 0, 0, 5, 0, 1, 0,
					1, 5, 4, 0, 0, 0, 0, 0, 0,
					2, 0, 0, 0, 7, 0, 9, 0, 8,
					8, 0, 0, 0, 1, 0, 0, 0, 3,
					0, 0, 0, 0, 0, 0, 8, 0, 0,
					0, 0, 7, 0, 0, 6, 0, 2, 0 };

	printMatSuduku(a);
	bool resFlase1 = SolveSoduku(a, 0, 0, false);
	printMatSuduku(a);

	int b[9][9] = {	0, 1, 0, 5, 2, 0, 0, 6, 0,
					0, 0, 3, 0, 0, 0, 0, 5, 2,
					0, 4, 0, 7, 0, 9, 0, 0, 1,
					0, 0, 0, 0, 0, 5, 0, 1, 0,
					1, 0, 4, 0, 0, 0, 0, 0, 0,
					2, 0, 0, 0, 7, 0, 9, 0, 0,
					0, 0, 0, 0, 1, 0, 0, 0, 3,
					0, 0, 0, 0, 0, 0, 0, 0, 0,
					0, 0, 7, 0, 0, 0, 0, 2, 0 };

	printMatSuduku(b);
	bool resFlase2 = SolveSoduku(b, 0, 0, false);
	printMatSuduku(b);

	int c[9][9] = { 0, 1, 0, 0, 2, 0, 0, 6, 0,
					0, 0, 3, 0, 0, 0, 0, 0, 0,
					0, 0, 0, 7, 0, 0, 0, 0, 1,
					0, 0, 0, 0, 0, 5, 0, 0, 0,
					1, 0, 4, 0, 0, 0, 0, 0, 0,
					0, 0, 0, 0, 0, 0, 9, 0, 0,
					0, 0, 0, 0, 1, 0, 0, 0, 3,
					0, 0, 0, 0, 0, 0, 0, 0, 0,
					0, 0, 7, 0, 0, 0, 0, 2, 0 };

	printMatSuduku(c);
	bool resFalse3 = SolveSoduku(c, 0, 0, false);
	printMatSuduku(c);


	/*int d[9][9] = { 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 1,
	0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 9, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 7, 0, 0, 0, 0, 0, 0 };

	printMatSuduku(d);
	bool resTruel = solvesoduku(d, 0, 0, true);
	printmatsuduku(d);*/

	cout << endl << "Matrix Section - BFS" << endl;

	// init Mat
	Mat mat[N][N];
	SetValue(mat, 0, N - 1, 0, N - 1, NO);
	SetVisited(mat, 0, N - 1, 0, N - 1, WHITE);
	SetParent(mat, 0, N - 1, 0, N - 1, -1, -1);

	//Page 10

	// Create Specific Scenario
	SetValue(mat, 2, 5, 1, 1, YES);
	SetValue(mat, 1, 2, 2, 4, YES);
	mat[0][3].value = YES;
	mat[3][5].value = YES;
	mat[4][6].value = YES;
	mat[2][6].value = YES;
	mat[1][7].value = YES;
	SetValue(mat, 1, 2, 2, 4, YES);
	SetValue(mat, 5, 5, 4, 5, YES);
	SetValue(mat, 6, 6, 2, 5, YES);

	printMatValue(mat);
	printMatVisited(mat);
	printMatParent(mat);

	BFS(mat, 6, 2);
	printMatValue(mat);
	printMatVisited(mat);
	printMatParent(mat);

	printBFS(mat, 6, 2, 6, 5);
	printBFS(mat, 6, 2, 1, 2);

	return 0;
}


void printMatValue(Mat mat[][N])
{
	cout << endl << "Mat values:" << endl;
	for (int i = 0; i < N; i++)
	{
		for (int j = 0; j < N; j++)
		{
			cout << mat[i][j].value << ",";
		}
		cout << endl;
	}
}


void printMatVisited(Mat mat[][N])
{
	cout << endl << "Mat visited:" << endl;
	for (int i = 0; i < N; i++)
	{
		for (int j = 0; j < N; j++)
		{
			cout << mat[i][j].visited << ",";
		}
		cout << endl;
	}
}


void printMatParent(Mat mat[][N])
{
	cout << endl << "Mat Parent Row:" << endl;
	for (int i = 0; i < N; i++)
	{
		for (int j = 0; j < N; j++)
		{
			cout << mat[i][j].parentRow << ",";
		}
		cout << endl;
	}



	//Page 11

	cout << endl << "Mat Parent col:" << endl;
	for (int i = 0; i < N; i++)
	{
		for (int j = 0; j < N; j++)
		{
			cout << mat[i][j].parentCol << ",";
		}
		cout << endl;
	}
}


void printAnyMat(vector<vector<int>> mat)
{

	cout << endl;

	for (int i = 0; i < mat.size(); i++)
	{
		for (int j = 0; j < mat[i].size(); j++)
		{
			cout << mat[i][j] << ",";
		}
		cout << endl;
	}

}


void printAnyMatXY(vector<vector<int>> mat)
{

	cout << endl;

	for (int i = mat.size() - 1; i >= 0; i--) // row = y 
	{
		for (int j = 0; j < mat[i].size(); j++) // col = x
		{
			cout << mat[i][j] << ",";
		}
		cout << endl;
	}

}

// including rowstart and rowEnd
void SetVisited(Mat mat[][N], int rowstart, int rowEnd, int colstart, int colEnd, NodeMarker visited)
{
	for (int i = rowstart; i <= rowEnd; i++)
	{
		for (int j = colstart; j <= colEnd; j++)

			mat[i][j].visited = visited;
	}
}


// including rowstart and rowEnd
void SetValue(Mat mat[][N], int rowstart, int rowEnd, int colstart, int colEnd, int value)
{
	for (int i = rowstart; i <= rowEnd; i++)
	{
		for (int j = colstart; j <= colEnd; j++)

			mat[i][j].value = value;

	}


	//Page 12

}


// including rowstart and rowEnd
void SetParent(Mat mat[][N], int rowstart, int rowEnd, int colstart, int colEnd, int parentRow, int parentCol)
{
	for (int i = rowstart; i <= rowEnd; i++)
	{
		for (int j = colstart; j <= colEnd; j++)
		{
			mat[i][j].parentRow = parentRow;
			mat[i][j].parentCol = parentCol;
		}
	}
}


void BFS(Mat mat[][N], int SourceRow, int SourceCol)
{
	Node sourceNODE = { SourceRow, SourceCol };
	queue<Node> queueNodes;

	mat[SourceRow][SourceCol].visited = GRAY;
	queueNodes.push(sourceNODE); // unvisited Nodes Queue by BFS order

	while (!queueNodes.empty())
	{ // while not emepty

		Node curNode = queueNodes.front(); // Get the LIFO elemet (without erasing)
		queueNodes.pop(); // Erase the element

						  // Insert all unvisited nei to queue
		for (int deltaRow = -1; deltaRow <= 1; deltaRow++)
		{
			for (int deltacol = -1; deltacol <= 1; deltacol++)
			{
				int neiRow = curNode.row + deltaRow;
				int neiCol = curNode.col + deltacol;
				Node neiNode = { neiRow, neiCol };

				if (neiRow < N && neiRow >= 0 && neiCol < N && neiCol >= 0 && // ligimate nei candidate in borders
					(mat[neiRow][neiCol].value == YES) && //This is nei indeed
					(mat[neiRow][neiCol].visited == WHITE))
					// unmarked - not visited yet
				{
					mat[neiRow][neiCol].visited = GRAY;
					mat[neiRow][neiCol].parentRow =
						curNode.row;
					mat[neiRow][neiCol].parentCol =
						curNode.col;

					queueNodes.push(neiNode);
				}
			}
		}

		mat[curNode.row][curNode.col].visited = BLACK;

	}
}


void printBFS(Mat matorigional[][N], int sourceRow, int sourceCol, int destRow, int destCol)

//Page 13

{
	Mat matPrint[N][N]; // preserving original mat (not destroying)
	for (int i = 0; i < N; i++)
	{
		for (int j = 0; j < N; j++)
		{
			matPrint[i][j] = matorigional[i][j];
		}
	}

	cout << endl << "printBFS:" << endl;

	int curRow = destRow;
	int curCol = destCol;
	while ( //!(curRow == sourceRow && curCol == sourceCol) && 
		curRow != -1 && curCol != -1)
	{   //cout « endl « curRow « curcol;
		matPrint[curRow][curCol].value += BFSAdd;
		int nextRow = matPrint[curRow][curCol].parentRow;
		int nextCol = matPrint[curRow][curCol].parentCol;

		curRow = nextRow;
		curCol = nextCol;
	}
	//matPrint[sourceRow][sourcecol].value += BFSAdd;

	printMatValue(matPrint);
}


Location NextFirstUnassignedLocation(int a[][9], int x, int y)
{
	for (int r = x, j = y; r < 9; r++)
	{
		for (int c = j; c < 9; c++)
		{
			if (a[r][c] == 0) {
				return{ r, c };
			}
		}

		j = 0;
	}

	return{ -1, -1 };
}


bool isLegal(int a[][9], int x, int y, int num)
{
	for (int i = 0; i < 9; i++)
	{
		if ((a[i][y] == num) || (a[x][i] == num))
		{
			return false;
		}
	}

	for (int i = x / 3; i < x / 3 + 3; i++)
	{
		for (int j = y / 3; j < y / 3 + 3; j++)
		{
			if (a[i][j] == num) {
				return false;
			}
		}
	}

	//Page 14

	return true;
}


bool SolveSoduku(int a[][9], int x, int y, bool print)
{

	Location next = NextFirstUnassignedLocation(a, x, y);

	int x_next = next.x;
	int y_next = next.y;
	if (x_next == -1 && y_next == -1)
	{
		return true;
	}

	for (int num = 1; num <= 9; num++)
	{
		if (isLegal(a, x_next, y_next, num))
		{
			a[x_next][y_next] = num;

			bool res = SolveSoduku(a, x_next, y_next, print);

			if (res == true) {
				return true;
			}

			// else 
			a[x_next][y_next] = 0;
		}

		if (print) {
			printMatSuduku(a);
		}
	}

	return false;
}


void printMatSuduku(int a[][9])
{
	cout << endl << "suduku matrix values:" << endl;
	for (int i = 0; i < 9; i++)
	{
		for (int j = 0; j < 9; j++)
		{
			cout << a[i][j] << ",";
		}
		cout << endl;
	}
}



void drawCircle(int a, int b, int r, vector<vector<int>> & mat, int value)
{

	mat[a + r][b] = value;
	mat[a - r][b] = value;
	mat[a][b + r] = value;
	mat[a][b - r] = value;

	for (int i = 1; i < r; i++)
	{
		// choose y
		// int y = b+i;


		//Page 15

		// find x given y,a,b,r:
		// (x-a)A2 + (y-b)A2 = rA2 ==> x = round{ a (+-)sqrt(rA2-(y-b)A2) }, y = round{ b (+-)sqrt(rA2-(x-a)A2) } 
		double semi = sqrt(r*r - i * i);

		int x1 = 0.5 + a + semi; // 0.5 for rounding
		int x2 = 0.5 + a - semi; // 0.5 for rounding
		int y1 = 0.5 + b + semi; // 0.5 for rounding
		int y2 = 0.5 + b - semi; // 0.5 for rounding

		mat[x1][b + i] = value;
		mat[x2][b + i] = value;
		mat[x1][b - i] = value;
		mat[x2][b - i] = value;

		mat[b + i][y1] = value;
		mat[b + i][y2] = value;
		mat[b - i][y1] = value;
		mat[b - i][y2] = value;

	}

}



// Assume x2>=x1, y2>=yl
void drawLine(int x1, int x2, int y1, int y2, vector<vector<int>> & mat, int value)
{
	///////////////////////////////////////// 
	///// Treat negative angle - Start ///// 
	///////////////////////////////////////// 
	if (x1 > x2)
	{
		//swap x1 x2 
		int tmp = x1;
		x1 = x2;
		x2 = tmp;
		//swap y1, y2
		tmp = y1;
		y1 = y2;
		y2 = tmp;
	}

	int d = 1;
	if (y2 < y1)
	{
		d = -1;
	}
	///////////////////////////////////////// 
	///// Treat negative angle - End    ///// 
	/////////////////////////////////////////

	float sinAlphaSquared;
	float curSinAlphaSquared = 0.5;

	mat[y1][x1] = value;
	mat[y2][x2] = value;

	sinAlphaSquared = SinAlphaSquared(x1, x2, y1, y2);
	int curX = x1;
	int curY = y1;

	while (!((abs(x2 - curX) <= 1) && (abs(y2 - curY) <= 1)))
	{
		// Decide next point of line.
		if (sinAlphaSquared == 1)


			//Page 16

		{
			curY += d;
		}
		else if (sinAlphaSquared == 0)
		{
			curX += 1;
		}
		else if (curSinAlphaSquared < sinAlphaSquared)
		{
			curY += d;
		}
		else
		{
			curX += 1;
		}

		// Fill next point (line).
		mat[curY][curX] = value; // curY is rows, curX is col

								 // Update cur cosAlphaSquared.
		curSinAlphaSquared = SinAlphaSquared(x1, curX, y1, curY);

	}
}


float CosAlphaSquared(int x1, int x2, int y1, int y2)
{
	float lengthX = float(x2) - float(x1);
	float lengthY = float(y2) - float(y1);

	if (lengthX == 0 && lengthY == 0)
	{
		return 0;
	}

	// float cosAlpha = lengthx / sqrt(lengthx*lengthx + lengthY*lengthY); 

	return lengthX * lengthX / (lengthX*lengthX + lengthY * lengthY);
}


float SinAlphaSquared(int x1, int x2, int y1, int y2)
{
	return (1 - CosAlphaSquared(x1, x2, y1, y2));
}


void drawPolygon(Polygon & poly, vector<vector<int>> & mat, int value)
{
	int PolySize = poly.points2D.size();

	for (int i = 0; i < PolySize; i++)
	{
		// Extract pair of points that construct a lines 
		Point2D p1 = poly.points2D[i];
		Point2D p2 = poly.points2D[(i + 1) % PolySize];

		drawLine(p1.x, p2.x, p1.y, p2.y, mat, value);
	}
}

void drawPolygon(Polygon & poly, vector<vector<int>> & mat, vector<float> value)
{
	int PolySize = poly.points2D.size();

	for (int i = 0; i < PolySize; i++)
	{

		//Page 17

		// Extract pair of points that construct a line 
		Point2D pl = poly.points2D[i];
		Point2D p2 = poly.points2D[(i + 1) % PolySize];

		drawLine(pl.x, p2.x, pl.y, p2.y, mat, (int)value[i % value.size()]);
	}
}


/*

A

bl I I     I
b0 I
I           >
 a0    al
 */
BBOX getBBOX(Polygon poly)
{
	int a0 = poly.points2D[0].x;
	int al = poly.points2D[0].x;
	int b0 = poly.points2D[0].y;
	int bl = poly.points2D[0].y;

	//for each (Point2D p in poly.points2D)
	for (Point2D p : poly.points2D)
	{
		if (p.x < a0)
		{
			a0 = p.x;
		}
		else if (p.x > al)
		{
			al = p.x;
		}

		if (p.y < b0)
		{
			b0 = p.y;
		}
		else if (p.y > bl)
		{
			bl = p.y;
		}
	}

	return{ a0, al, b0, bl };
}


void drawBBOX(BBOX bbox, vector<vector<int>> & m, int value)
{
	Polygon polyBBOX({ { bbox.a0, bbox.b0 },{ bbox.a1, bbox.b0 },{ bbox.a1, bbox.b1 },{ bbox.a0, bbox.b1 } });
	drawPolygon(polyBBOX, m, value);
}


int InOutPolygon(int x, int y, Polygon poly)
{
	BBOX bbox = getBBOX(poly);
	int a0 = bbox.a0;
	int a1 = bbox.a1;
	int b0 = bbox.b0;
	int b1 = bbox.b1;

	//Page 18

	if (x<a0 || x>a1 || y<b0 || y>b1)
	{
		return 0;
	}

	int PolySize = poly.points2D.size();

	for (int i = 0; i < PolySize; i++)
	{
		// Extract pair of points that construct a line 
		Point2D pl = poly.points2D[1];
		Point2D p2 = poly.points2D[(i + 1) % PolySize];

		//////// Is it the line.
		if (pl.x == p2.x)
		{
			if ((x == pl.x) &&
				((y <= p2.y && pl.y <= y) || (y <= pl.y && p2.y <= y))
				)
			{
				return 2;
			}
		}
		else
		{
			float slope = (float(pl.y) - float(p2.y)) / (float(pl.x) - float(p2.x));
			float bias = float(pl.y) - slope * float(pl.x);
			float yLinear = slope * x + bias; // yLinear = ax + b

			if (int(yLinear + 0.5) == y) // round(yLinear) =? y 
			{
				return 2;
			}
		}

	}

	int count = 0;
	/*
		TBD: Incomplete
	*/

	return 0;
}


float d(float x1, float y1, LineFormula lineEq)
{
	float d = abs(lineEq.A*x1 + lineEq.B*y1 + lineEq.C) /
		sqrt(lineEq.A*lineEq.A + lineEq.B*lineEq.B);
	return d;
}


// Ax + By + C = 0
LineFormula getLineFormula(float x1, float y1, float x2, float y2)
{
	/*
	Given:
	Ax1 + By1 + c = 0
	Ax2 + By2 + C = 0
	So:
	Ax1 + By1 + C = 0

	Ax2 + By2 + C = 0
	------------------
	//Page 19

	A(x1-x2) + B(y1-y2) + C - C = 0
	A(x1-x2) + B(y1-y2) = 0

	Choose A = (yl-y2) --> B = -(x2-x1)
	C = -Ax1 - By1 (or C = -Ax2 - By2)
	*/

	float A = y2 - y1;
	float B = -(x2 - x1);
	float C = -A * x1 - B * y1; // or C = -A*x2 - B*y2 

	return { A, B, C };
}


Point2D arbitraryPointInsideTriangule(Triangule triangule)
{
	float x_aveline01 = (triangule.points2D[0].x + triangule.points2D[1].x)
		/ 2;
	float y_aveline01 = (triangule.points2D[0].y + triangule.points2D[1].y)
		/ 2;

	float xInside = (x_aveline01 + triangule.points2D[2].x) / 2;
	float yInside = (y_aveline01 + triangule.points2D[2].y) / 2;

	return { (int)xInside, (int)yInside };
}


void drawPoint(Point2D p, vector<vector<int>> & mat, int value)
{
	mat[p.y][p.x] = value;
}


// 0 - outside, 1 - inside, 2 - on edge
int InOutTriangule(Triangule triangule, Point2D pQuery)
{

	LineFormula lineEq01 = getLineFormula(triangule.points2D[0].x,
		triangule.points2D[0].y, triangule.points2D[1].x, triangule.points2D[1].y);
	LineFormula lineEq12 = getLineFormula(triangule.points2D[1].x,
		triangule.points2D[1].y, triangule.points2D[2].x, triangule.points2D[2].y);
	LineFormula lineEq02 = getLineFormula(triangule.points2D[0].x,
		triangule.points2D[0].y, triangule.points2D[2].x, triangule.points2D[2].y);

	float signQuery01 = lineEq01.A*pQuery.x + lineEq01.B*pQuery.y + lineEq01.C;
	float signQuery12 = lineEq12.A*pQuery.x + lineEq12.B*pQuery.y + lineEq12.C;
	float signQuery02 = lineEq02.A*pQuery.x + lineEq02.B*pQuery.y + lineEq02.C;

	// float epsilon =2;
	float epsilon01 = (abs(lineEq01.A) + abs(lineEq01.B)) / 2;
	float epsilon12 = (abs(lineEq12.A) + abs(lineEq12.B)) / 2;
	float epsilon02 = (abs(lineEq02.A) + abs(lineEq02.B)) / 2;

	if ((abs(signQuery01) <= epsilon01 && pQuery.x >=
		min(triangule.points2D[0].x, triangule.points2D[1].x) && pQuery.x <=
		max(triangule.points2D[0].x, triangule.points2D[1].x) && pQuery.y >=
		min(triangule.points2D[0].y, triangule.points2D[1].y) && pQuery.y <=
		max(triangule.points2D[0].y, triangule.points2D[1].y))

		||

		(abs(signQuery12) <= epsilon12 && pQuery.x >=
			min(triangule.points2D[1].x, triangule.points2D[2].x) && pQuery.x <=
			max(triangule.points2D[1].x, triangule.points2D[2].x) && pQuery.y >=
			min(triangule.points2D[1].y, triangule.points2D[2].y) && pQuery.y <=
			max(triangule.points2D[1].y, triangule.points2D[2].y))

		||

		//Page 20

		(abs(signQuery02) <= epsilon02 && pQuery.x >=
			min(triangule.points2D[0].x, triangule.points2D[2].x) && pQuery.x <=
			max(triangule.points2D[0].x, triangule.points2D[2].x) && pQuery.y >=
			min(triangule.points2D[0].y, triangule.points2D[2].y) && pQuery.y <=
			max(triangule.points2D[0].y, triangule.points2D[2].y))
		)
	{
		return 2;
	}

	Point2D p = arbitraryPointInsideTriangule(triangule);
	float sign01 = lineEq01.A*p.x + lineEq01.B*p.y + lineEq01.C;
	float sign12 = lineEq12.A*p.x + lineEq12.B*p.y + lineEq12.C;
	float sign02 = lineEq02.A*p.x + lineEq02.B*p.y + lineEq02.C;

	if (SameSign(sign01, signQuery01) && SameSign(sign12, signQuery12) &&
		SameSign(sign02, signQuery02))
	{
		return 1;
	}

	return 0;

}


// true - both are non-zero and same sign, false - different sign or one is zero 
bool SameSign(float a, float b)
{
	if (a*b > 0)
	{
		return true;
	}

	return false;
}


void TestInOutTriangule(Triangule triangule, vector<vector<int>> mat)
{
	printAnyMat(mat);

	for (int i = 0; i < mat.size(); i++) // y = row
	{
		for (int j = 0; j < mat[i].size(); j++) // x = col
		{
			drawPoint({ j, i }, mat, InOutTriangule(triangule, { j, i }));
		}
		//printAnyMat(mat);
	}

	printAnyMat(mat);
}


bool isonLineSegment(LineSegment lineSeg, Point2D pQuery)
{
	return isOnLineSegment(lineSeg.points2D[0], lineSeg.points2D[1], pQuery);
}


bool isOnLineSegment(Point2D p1, Point2D p2, Point2D pQuery)
{
	LineFormula lineEq01 = getLineFormula(p1.x, p1.y, p2.x, p2.y);

	float signQuery = lineEq01.A*pQuery.x + lineEq01.B*pQuery.y +

		//Page 21

		lineEq01.C;

	float epsilon = (abs(lineEq01.A) + abs(lineEq01.B)) / 2;

	if (abs(signQuery) <= epsilon && pQuery.x >= min(p1.x, p2.x) &&
		pQuery.x <= max(p1.x, p2.x) && pQuery.y >= min(p1.y, p2.y) && pQuery.y <=
		max(p1.y, p2.y))
	{
		return true;
	}

	return false;
}


void TestIsOnLineSegment(LineSegment lineSeg, vector<vector<int>> mat)
{
	printAnyMat(mat);

	for (int i = 0; i < mat.size(); i++) // y = row
	{
		for (int j = 0; j < mat[i].size(); j++)  // x = col
		{
			drawPoint({ j, i }, mat,
				isOnLineSegment(lineSeg.points2D[0], lineSeg.points2D[1], { j, i }) ? 1 : 0);
		}
		//printAnyMat(mat);
	}

	printAnyMat(mat);
}


void drawClockWisePolygon(Polygon poly, vector<vector<int>> & mat, int Method)
{
	vector<Point2D> clockWiseOrderpPolygon = poly.GetPolygonClockWiseOrder(Method);

	int polySize = clockWiseOrderpPolygon.size();

	for (int i = 0; i < polySize; i++)
	{
		Point2D p1 = clockWiseOrderpPolygon[i];
		Point2D p2 = clockWiseOrderpPolygon[(i + 1) % polySize];

		//drawPoint(pl, mat, i + 1);
		drawLine(p1.x, p2.x, p1.y, p2.y, mat, i + 1);
	}

}


void drawCounterClockWisePolygon(Polygon poly, vector<vector<int>> & mat, int Method)
{
	vector<Point2D> counterClockWiseOrderPolygon = poly.GetPolygonCounterClockWiseOrder(Method);

	int polysize = counterClockWiseOrderPolygon.size();

	for (int i = 0; i < polysize; i++)
	{
		Point2D p1 = counterClockWiseOrderPolygon[i];
		Point2D p2 = counterClockWiseOrderPolygon[(i + 1) % polysize];

		//drawPoint(p1, mat, i + 1);
		drawLine(p1.x, p2.x, p1.y, p2.y, mat, i + 1);

	}
}
//Page 22



// Returning Counter Clock wise angle from line 1 to line 2.
float CalcAngleBetweenLineSegments(LineSegment lineSegment1, LineSegment lineSegment2, int DegRad)
{
	// v =    xl-x0 ,y1-y0     for each line segment
	vector<float> v1 = { float(lineSegment1.points2D[1].x) -
							float(lineSegment1.points2D[0].x), float(lineSegment1.points2D[1].y) -
							float(lineSegment1.points2D[0].y) };

	vector<float> v2 = { float(lineSegment2.points2D[1].x) -
							float(lineSegment2.points2D[0].x), float(lineSegment2.points2D[1].y) -
							float(lineSegment2.points2D[0].y) };

	// angleRad = v1v2 / |v1||v2|
	// float cosAngleRad = inner_product(v1,v2);

	float dot = v1[0] * v2[0] + v1[1] * v2[1];		// x1* x2 + yl*y2
	float det = v1[0] * v2[1] - v1[1] * v2[0];		// x1*y2 - y1*x2
	float angleRad = atan2(det, dot);				// Counter clock wise angle

	if (angleRad < 0)
	{
		angleRad += 2 * PI;
	}

	// Returning the angle in the desired units(Degrees / Radians).
	if (DegRad == 1)
	{
		return angleRad * 360 / (2 * PI); // angle Degrees
	}

	return angleRad;
}


void TestGetSubPolygon()
{
	vector<Point2D> vect2D = { { 3, 5 } ,{ 4, 26 },{ 13, 24 },{ 27, 28 }, { 17, 11 },{ 17, 2 },{ 10, 3 } };
	Polygon nonConvexPolyExample(vect2D);

	try
	{
		Polygon subPolygon = nonConvexPolyExample.GetSubPolygon({ 1, 2, (int)nonConvexPolyExample.points2D.size() });
		const char* s = subPolygon.IsConcavePolygon() ? "false" : "true";
		cout << "IsConcavePolygon = " << s << endl;
	}
	catch (exception ex)
	{
		cout << "GetSubPolygon exception: " << ex.what() << endl;
	}

	try
	{
		Polygon subPolygon = nonConvexPolyExample.GetSubPolygon({ 1, 2, 3 });
		const char* s = subPolygon.IsConcavePolygon() ? "false" : "true";
		cout << "IsConcavePolygon = " << s << endl;
	}
	catch (exception ex)
	{
		cout << "GetsubPolygon exception: " << ex.what() << endl;
	}


	try
	{
		Polygon subPolygon = nonConvexPolyExample.GetSubPolygon({ -1, 2,

			//Page 23

			3 });
		const char* s = subPolygon.IsConcavePolygon() ? "false" : "true";
		cout << "IsConcavePolygon = " << s << endl;
	}
	catch (exception ex)
	{
		cout << "GetSubPolygon exception: " << ex.what() << endl;
	}

	try
	{
		Polygon subPolygon = nonConvexPolyExample.GetSubPolygon({ 0, 2, 3 });
		const char* s = subPolygon.IsConcavePolygon() ? "false" : "true";
		cout << "IsConcavePolygon = " << s << endl;
	}
	catch (exception ex)
	{
		cout << "GetSubPolygon exception: " << ex.what() << endl;
	}
}


void printVector(vector<int> vect)
{
	if (vect.size() == 0) {
		return;
	}

	cout << vect[0];
	for (int i = 1; i < vect.size(); i++)
	{
		cout << "," << vect[i];
	}
	cout << endl;
}


void printIntList(list<int> myIntList)

{
	for (list<int>::iterator it = myIntList.begin(); it != myIntList.end(); it++)
	{
		cout << *it << ",";
	}
	cout << endl;
}


list<int> vectorIntToIntList(vector<int> intVector)
{
	list<int> intList;

	for (int i = 0; i < intVector.size(); i++)
	{
		intList.emplace_back(intVector[i]);
	}

	return intList;
}


vector<int> listIntToVectorList(list<int> intList)
{
	vector<int> intVector;
	int i = 0;

	for (list<int>::iterator it = intList.begin(); it != intList.end(); it++)
	{
		//Page 24

		intVector[i] = *it;
		i++;
	}

	return intVector;
}


void TestPolygonTriangulation()
{
	vector<Point2D> vect2D = { { 3, 5 },{ 4, 26 },{ 13, 24 },{ 27, 28 }, { 17, 11 },{ 17, 2 },{ 10, 3 } };
	Polygon nonConvexPolyExample(vect2D);

	vector<Triangule> polygonTriangulation = nonConvexPolyExample.PolygonTriangulation();

	vector<vector<int>> matTriangulation(30, vector<int>(30));
	drawPolygon(nonConvexPolyExample, matTriangulation);
	printAnyMatXY(matTriangulation);

	matTriangulation = vector<vector<int>>(30, vector<int>(30)); // "clear screen"
	//printAnyMatXY(matTriangulation);

	int color = 1;
	//for each (Triangule triangule in polygonTriangulation)
	for (Triangule triangule : polygonTriangulation)
	{
		drawPolygon(triangule, matTriangulation, color++);
		//printAnyMatXY(matTriangulation);
	}
	printAnyMatXY(matTriangulation);


	for (int i = 0; i < nonConvexPolyExample.points2D.size(); i++)

	{   
		// 0. create i shift vector for this iteration
		vector<int> shiftedVect(nonConvexPolyExample.points2D.size());

		for (int j = 0; j < shiftedVect.size(); shiftedVect[(i + j) % shiftedVect.size()] = j, j++);

		cout << endl;

		printVector(shiftedVect);

		// 1. Perform polygon triangulation
		polygonTriangulation = nonConvexPolyExample.GetSubPolygon(shiftedVect).PolygonTriangulation();

		// 2. Draw polygon triangulation result
		matTriangulation = vector<vector<int>>(30, vector<int>(30)); // "clear screen"

		int color = 1;
		// for each (Triangule triangule in polygonTriangulation)
		for (Triangule triangule : polygonTriangulation)
		{
			drawPolygon(triangule, matTriangulation, color++);
			//printAnyMatXY(matTriangulation);
		}
		printAnyMatXY(matTriangulation);
	}

	vector<vector<Point2D>> points2Dvectors = {
												{ { 3,  5 },{ 4, 26 },{ 13,  24 },{ 27,  28 },{ 17,  11 },{ 17, 2 },{ 10,  3 } },
												{ { 674,   285 },{ 635,  325 },{ 600,  147 },{ 563, 324 },{ 631, 145 },{ 505,  328 },{ 442,  152 },{ 440,  375 },{ 336,  336 },{ 402,  417 },{ 458,  450 },{ 473,  575 },{ 656,  564 },{ 655,  350 } },
												//{{}, 11, fl, fl, fl, 0, {} 1


												//Page 25

												//{{}, {},  0,  {}, {}, {}, {} 1
												//{{}, fl,  {}, {}, {}, 0,  {} }
	};

	float scalar = float(1) / float(20);
	points2Dvectors[1] = scale(points2Dvectors[1], scalar);
	BBOX bbox = getBBOX(Polygon(points2Dvectors[1]));
	points2Dvectors[1] = translate(points2Dvectors[1], { -bbox.a0 + 4, -bbox.b0 + 4 });

	// for each (vector<Point2D> vect in points2Dvectors)
	for (vector<Point2D> vect : points2Dvectors)
	{
		matTriangulation = vector < vector <int>>(30, vector<int>(30)); // "clear screen"
		printAnyMatXY(matTriangulation);

		Polygon poly(vect);
		drawPolygon(poly, matTriangulation);
		printAnyMatXY(matTriangulation);

		matTriangulation = vector<vector<int>>(30, vector<int>(30)); // "clear screen"
		printAnyMatXY(matTriangulation);

		vector<Triangule> polygonTriangulation = poly.PolygonTriangulation();
		vector<Polygon> polygonTriangulationconvertedvector(polygonTriangulation.size());
		for (int i = 0; i < polygonTriangulation.size();
			polygonTriangulationconvertedvector[i] = polygonTriangulation[i],
			i++);

		drawMultiplePolygons(polygonTriangulationconvertedvector, matTriangulation);
		printAnyMatXY(matTriangulation);
	}

	int dummy = 0;
}


void drawMultipleTriangules(vector<Triangule> triangulesVector, vector<vector<int>>& mat)
{
	int color = 0;
	// for each (Triangule poly in triangulesVector)
	for (Triangule poly : triangulesVector)
	{
		drawPolygon(poly, mat, (color % 9) + 1);
		color++;
		//printAnyMatXY(matTriangulation);
	}
	//printAnymatxY(mat);
}


void drawMultiplePolygons(vector<Polygon> polygonsVector, vector<vector<int>>& mat)
{
	int color = 0;
	// for each (Polygon poly in polygonsVector)
	for (Polygon poly : polygonsVector)
	{
		drawPolygon(poly, mat, (color % 9) + 1);
		color++;
		//printAnyMatXY(matTriangulation);
	}
	//printAnymatxY(mat);
}


//Page 26

vector<Point2D> translate(vector<Point2D> vectPoints2D, Point2D move)
{
	vector<Point2D> result = vectPoints2D;

	for (int i = 0; i < result.size(); i++)
	{
		result[i].x += move.x;
		result[i].y += move.y;
	}

	return result;
}


// Scaling all points
// No interpolation of new points nor decimation of intersection produced. 
vector<Point2D> scale(vector<Point2D> vectPoints2D, float scalar)
{
	vector<Point2D> result = vectPoints2D;

	for (int i = 0; i < result.size(); i++)
	{
		result[i].x = (int)(float(result[i].x) * scalar);
		result[i].y = (int)(float(result[i].y) * scalar);
	}

	return result;
}



//void optimalRout(id, routingTable)
//{
//
//}



float Polygon::SumOfInteriorAngles(int DegRad)
{
	float triangulesumAngles;
	if (DegRad == 1)
	{ // degrees
		triangulesumAngles = 180;
	}
	else
	{ // radians
		triangulesumAngles = PI;
	}

	return (points2D.size() - 2) * triangulesumAngles; // (n-2)*180
}


// always equals 360 (for any n sides polygon) 
float Polygon::SumOfExteriosAngles(int DegRad)
{
	//// methodl
	//if (DegRad == 1){ 
	// return 360;
	//}
	//return 2*PI;

	// Method 2
	float straightLineAngle;
	if (DegRad == 1)
	{ // degrees


	  //Page 27

		straightLineAngle = 180;
	}
	else
	{ // radians
		straightLineAngle = PI;
	}

	float averageinteriorAngle = SumOfInteriorAngles(DegRad) / points2D.size();

	return (straightLineAngle - averageinteriorAngle) * points2D.size();
}


float Polygon::SumOfInteriorComplementaryAngles(int DegRad)
{   
	//// methodl
	//if (DegRad == 1) {
	//   return 360 * points2D.size() - SumOfInteriorAngles(DegRad);  // Deg
	// }
	//return 2 * PI * points2D.size() - SumOfInteriorAngles(DegRad); // Rad

	// Method 2
	float Full360angle;
	if (DegRad == 1)
	{ // degrees
		Full360angle = 360;
	}
	else
	{ // radians
		Full360angle = 2 * PI;
	}

	float averageInteriorAngle = SumOfInteriorAngles(DegRad) / points2D.size();

	return (Full360angle - averageInteriorAngle) * points2D.size();
}

bool Polygon::IsClockWise(int Method)
{

	// Method 1 - Based on sum of areas 
	if (Method == 1)
	{

		int sum = 0;
		int PolySize = points2D.size();

		for (int i = 0; i < PolySize; i++)
		{
			// Extract pair of points that construct a line
			Point2D p1 = points2D[i];
			Point2D p2 = points2D[(i + 1) % PolySize];

			sum += (p2.x - p1.x)*(p2.y + p1.y);
		}

		if (sum > 0)
		{
			return true; // clockwise
		}

		return false; // counter clockwise
	}


	//Page 28

	// Method 2 - Based on finding most buttom vertex and routing to the most right vertex.
	if (Method == 2)
	{
		int pBottomVertexIndex = getBottomVertexIndex();
		int pRightRelateBottomVertexIndex = GetRightmostVertexIndex(pBottomVertexIndex); // This is the Counter Clock Wise order
		if (pRightRelateBottomVertexIndex > pBottomVertexIndex) // points2D are arranged counterclockwise order
		{
			return false;
		}

		return true;
	}


	/* Method3 - Based on calculating sum of counter clock wise angles and
	comparing against expected n-polygon interior
	angles sum.*/
	int polySize = points2D.size();
	vector<float> polyAngles(points2D.size());

	for (int i = 0; i < polySize; i++)
	{
		Point2D p0 = points2D[((i - 1) + polySize) % polySize]; // for i = 0: (-1 + n) % n = n - 1
		Point2D p1 = points2D[i];
		Point2D p2 = points2D[(i + 1) % polySize]; // for i = n-1: n % n = 0

		LineSegment lineSeg1({ p1, p0 });
		LineSegment lineSeg2({ p1, p2 });

		//float angle = calcAngleBetweenLinesegments(lineseg1, lineSeg2, DegRad);
		float angle = CalcAngleBetweenLineSegments(lineSeg2, lineSeg1);
		polyAngles[i] = angle; // polyg Angles are counter clock wise.
	}

	float sumPolyAngles = 0;
	for (int i = 0; i < polySize; i++)
	{
		sumPolyAngles += polyAngles[i];
	}

	/*   If counter clock wise angles sum equales the expected n polygons
	interior angles sum,
	then the order is counter clock wise.

	We notice that:

	sum of interior Angles for n sides polygon is: (n-2)*180
	Sum of interior complementary angles for n sides polygon is:
	360*n - (n-2)*180
	so, this method is incorrect (due to ambiguity) only when: 360*n
	- (n-2)*180 = (n-2)*180.
	--> 360*n = 2*(n-2)*180
	--> n = (n-2) --> for any n: 0=2

	so this method is always correct.

	*/

	float epsilonQuantizationError = 0.01*polySize; // Instead of cheking if sumPolyAngles == Sum0finteriorAngles()
	if (abs(sumPolyAngles - SumOfInteriorAngles()) < epsilonQuantizationError)
	{

		//Page 29

		return false; // counter clock wise order
	}
	return true;

}


vector<Point2D> Polygon::GetPolygonClockWiseOrder(int Method)
{
	vector<Point2D> points2DClockWise = points2D;

	if (!IsClockWise(Method))
	{
		reverse(points2DClockWise.begin(), points2DClockWise.end());
	}

	return points2DClockWise;

}


vector<Point2D> Polygon::GetPolygonCounterClockWiseOrder(int Method)
{
	vector<Point2D> points2DCounterClockWise = points2D;

	if (IsClockWise(Method))
	{
		reverse(points2DCounterClockWise.begin(), points2DCounterClockWise.end());
	}

	return points2DCounterClockWise;
}


int Polygon::getBottomVertexIndex()
{
	int PminY_Index = 0;

	for (int i = 1; i < points2D.size(); i++)
	{
		if (points2D[i].y < points2D[PminY_Index].y)
		{
			PminY_Index = i;
		}
	}

	return PminY_Index;
}


Point2D Polygon::getBottomVertex()
{
	return points2D[getBottomVertexIndex()];
}


int Polygon::GetRightmostVertexIndex(int pointIndex)
{
	LineSegment lineSegmentXAxis({ { 0, 0 },{ 1, 0 } }); // X axis
	LineSegment line1({ points2D[pointIndex], points2D[(pointIndex + 1) % points2D.size()] });

	// in order to get -1 index as 6 (cyclic).
	// (pointindex - 1 + points2D.size()) % points2D.size() in order to get (-1 + 7) % 7 = 6 and not - 1 % 7 = -1
	LineSegment line2({ points2D[pointIndex], points2D[(pointIndex - 1 + points2D.size()) % points2D.size()] });


	//Page 30

	float angle1 = CalcAngleBetweenLineSegments(lineSegmentXAxis, line1); // calc angle against X axis
	float angle2 = CalcAngleBetweenLineSegments(lineSegmentXAxis, line2); // calc angle against X axis

	if (angle1 < angle2) // line 1 is the right most
	{
		return (pointIndex + 1) % points2D.size();
	}

	return (pointIndex - 1 + points2D.size()) % points2D.size(); // line 2 is the right most
}


// Calculating Interior angles
// with correspondence to polygon's 2D points vector 
vector<float> Polygon::CalcPolygonAngles(int DegRad)
{
	vector<Point2D> points2DCounterClockWiseOrder = GetPolygonCounterClockWiseOrder();
	vector<float> polygonAngles(points2D.size());

	int polySize = points2DCounterClockWiseOrder.size();

	for (int i = 0; i < polySize; i++)
	{
		Point2D p0 = points2DCounterClockWiseOrder[((i - 1) + polySize) % polySize]; // for i = 0: (-1 + n) % n = n-1
		Point2D p1 = points2DCounterClockWiseOrder[i];
		Point2D p2 = points2DCounterClockWiseOrder[(i + 1) % polySize]; // for i = n-1: n % n = 0

		LineSegment lineSeg1({ p1, p0 });
		LineSegment lineSeg2({ p1, p2 });

		//float angle = CalcAngleBetweenLineSegments(lineSegl, lineseg2, DegRad);
		float angle = CalcAngleBetweenLineSegments(lineSeg2, lineSeg1, DegRad);
		polygonAngles[i] = angle; /* At this stage, polygonAngles are
								  counter clock wise order and should be flipped.*/
	}

	// keep correspondence to polygon's 2D points vector.
	if (IsClockWise()) /* polygon's 2D points order is Clock wise, hence
					   polygonAngles, which is counter clock wise should be flipped.*/
	{
		reverse(polygonAngles.begin(), polygonAngles.end());
	}

	return polygonAngles;
}


vector<bool> Polygon::ConvexVertices()
{
	vector<float> polyAngles = CalcPolygonAngles(1); // Degrees
	int polySize = polyAngles.size();
	vector<bool> convexVertices(polySize);

	for (int i = 0; i < polySize; i++)
	{
		convexVertices[i] = IsConvexVertex(polyAngles[i], 1); // Degrees
	}

	return convexVertices;

	//Page 31

}


// Concave vertex = Reflex vertex
vector<bool> Polygon::ConcaveVertices()
{
	vector<float> polyAngles = CalcPolygonAngles(1); // Degrees
	int polySize = polyAngles.size();
	vector<bool> concaveVertices(polySize);

	for (int i = 0; i < polySize; i++)
	{
		concaveVertices[i] = IsConcaveVertex(polyAngles[i], 1); // Degrees
	}

	return concaveVertices;
}


bool Polygon::IsConvexVertex(int index)
{
	vector<bool> convexVertices = ConvexVertices();
	return convexVertices[index];
}


bool Polygon::IsConcaveVertex(int index)
{
	vector<bool> concavevertices = ConcaveVertices();
	return concavevertices[index];
}


bool Polygon::IsConvexVertex(float angle, int DegRad)
{
	if (DegRad == 1) // Degrees units
	{
		return (angle < 180);
	}

	// Radians units
	return (angle < PI);
}


bool Polygon::IsConcaveVertex(float angle, int DegRad)
{
	if (DegRad == 1) // Degrees units
	{
		return (angle > 180);
	}

	// Radians units
	return (angle > PI);
}


bool Polygon::IsConvexPolygon()
{
	vector<bool> concaveVertices = ConcaveVertices();

	for (int i = 0; i < concaveVertices.size(); i++)
	{
		if (concaveVertices[i])
		{
			return false; // There is a concave (non-convex) vertex.
		}

		//Page 32

	}

	return true; // if we reach here, then all vertices are convex (non-concave).
}


bool Polygon::IsConcavePolygon()
{
	return !IsConvexPolygon();
}


int Polygon::NumberOfConvexVertices()
{
	vector<bool> convexVertex = ConvexVertices();
	int numberOfConvexVertices = 0;

	for (int i = 0; i < convexVertex.size(); i++)
	{
		if (convexVertex[i])
		{
			numberOfConvexVertices++;
		}
	}

	return numberOfConvexVertices;
}


int Polygon::NumberOfConcaveVertices()
{
	return (points2D.size() - NumberOfConvexVertices());
}


ConvexConcaveIndices Polygon::GetConvexConcaveIndices()
{
	vector<bool> convexVertex = ConvexVertices();
	vector<int> convexIndices(NumberOfConvexVertices());
	vector<int> concaveIndices(NumberOfConcaveVertices());

	int i_convex = 0;
	int i_concave = 0;

	for (int i = 0; i < convexVertex.size(); i++)
	{
		if (convexVertex[i])
		{
			convexIndices[i_convex] = i;
			i_convex++;
		}
		else
		{
			concaveIndices[i_concave] = i;
			i_concave++;
		}
	}

	return{ convexIndices , concaveIndices };

}


Polygon Polygon::GetSubPolygon(vector<int> points2DIndices)
{
	if (points2DIndices.size() > points2D.size())
	{


		//Page 33

		throw exception("GetSubPolygon() construction failure");
	}

	vector<Point2D> subPolygonPoints2D(points2DIndices.size());
	for (int i = 0; i < points2DIndices.size(); i++)
	{
		if (points2DIndices[i] >= points2D.size() || points2DIndices[i] < 0)
		{
			throw exception("GetSubPolygon() construction failure");
		}
		subPolygonPoints2D[i] = points2D[points2DIndices[i]];
	}

	Polygon subPolygon(subPolygonPoints2D);

	return subPolygon;
}

Polygon* Polygon::translatePolygon(Point2D move)
{
	points2D = translate(points2D, move);
	return this;
}

vector<Point2D> Polygon::GetReversePolygonVertices()
{
	vector<Point2D> vect = points2D;
	reverse(vect.begin(), vect.end());
	return vect;
}


vector<Point2D> Polygon::GetPolygonVertices()
{
	return points2D;
}

vector<Triangule> Polygon::PolygonTriangulation()
{
	//list<Point2D> points2Dlist(points2D.begin(), points2D.end());

	//Point2D x = points2Dlist.front(); // This is the begin
	//x = points2Dlist.front(); // Does not change
	//x = points2Dlist.front(); // Does not change

	//Point2D y = points2Dlist.back(); // This is the end
	//y = points2Dlist.back(); // Does not change
	//y = points2Dlist.back(); // Does not change


	vector<Triangule> vectorTriangules;
	// init remainingPointsindicesList to { 0, 1, 2, 3, points2D.size()-1 }
	list<int> remainingPointsIndicesList;
	for (int i = 0; i < points2D.size(); i++)
	{
		remainingPointsIndicesList.emplace_back(i);
	}

	//printIntList(points2DindicesList); // For testing

	vector<bool> convexVertices = ConvexVertices();
	vector<float> polygonAngles = CalcPolygonAngles();

	while (remainingPointsIndicesList.size() >= 3)


		//Page 34

	{
		//convexconcaveindices convexAndconcaveindices = GetConvexConcaveIndices();
		//list<int> convexindices = vectorIntToIntList(convexAndconcaveIndices.ConvexIndices);
		//list<int> concaveindices = vectorIntToIntList(convexAndconcaveIndices.ConcaveIndices);

		bool foundAnEar = false;
		int earIndex = -1;
		int earIndexPrev = -1;
		int earIndexNext = -1;
		list<int>::iterator earIterator;

		for (list<int>::iterator it = remainingPointsIndicesList.begin();
			it != remainingPointsIndicesList.end() && !foundAnEar; // didn't find an ear yet
			it++)
		{
			// 0. Get Pi out of points2D

			int index = *it; // points2Dlndex --> Get Pi out of points2D

			// 1. Check if vertex Pi is an Ear for remaining polygon

			if (convexVertices[index])
			{ // 1.1 this is a convex vertex for remaining polygon

			// 1.2 Get (P1-1, Pi, Pi+1)

				list<int>::iterator it_prev = it;
				list<int>::iterator it_next = it;

				if (it == remainingPointsIndicesList.begin())
				{
					it_prev = remainingPointsIndicesList.end();
					it_prev--;
					it_next++;
				}
				else if (it == remainingPointsIndicesList.end())
				{
					it_prev--;
					it_next = remainingPointsIndicesList.begin();
				}
				else
				{
					it_prev--;
					it_next++;
				}

				int index_prev = *it_prev;
				int index_next = *it_next;

				Triangule triangule({ points2D[index_prev], points2D[index], points2D[index_next] });

				// Iterate through all other remaining vertices and verify not
				bool isAnEar = true;

				for (list<int>::iterator it2 = remainingPointsIndicesList.begin();
					it2 != remainingPointsIndicesList.end()

					//Page 35

					&& isAnEar;
					it2++)
				{
					int index2 = *it2;

					if (it2 != it && it2 != it_prev && it2 != it_next)
					{ // same As: index2!= index && index2!=index_prev && index2!=index_next.
						if (InOutTriangule(triangule, points2D[index2]) != 0)
						{ // point2 is not outside

							isAnEar = false; // not an ear.

						}
					}
				}

				if (isAnEar)
				{
					foundAnEar = true;

					earIterator = it;
					earIndex = index; // ( = *it)
					earIndexPrev = index_prev;
					earIndexNext = index_next;
					vectorTriangules.push_back(triangule); // Alternatively { index_prev, index, index_next }
				}

			}

		}

		// updating The polygon

		if (foundAnEar) // According to 2-Ears-Theorem this is true at this stage of the algorithm
		{
			vector<float> earTrianguleAngles = vectorTriangules.back().CalcPolygonAngles();

			polygonAngles[earIndexPrev] -= earTrianguleAngles[0]; // index_prev
			polygonAngles[earIndexNext] -= earTrianguleAngles[2]; // index_next

			convexVertices[earIndexPrev] = IsConvexVertex(polygonAngles[earIndexPrev]);
			convexVertices[earIndexNext] = IsConvexVertex(polygonAngles[earIndexNext]);

			remainingPointsIndicesList.erase(earIterator); // delete point index
		}

	}


	return vectorTriangules;

}




//Page 36
