/**
 * @file shapeFlow.cpp
 * @brief ShapeFlow plugin for Maya
 * @section LICENSE The MIT License
 * @section  requirements:  Eigen library, Maya
 * @version 0.10
 * @date  01/Nov/2013
 * @author Shizuo KAJI
 */

#pragma comment(linker, "/export:initializePlugin /export:uninitializePlugin")

#include "StdAfx.h"

#include <maya/MFnPlugin.h>

#include <Eigen/Dense>
#include <numeric>

using namespace Eigen;
using namespace std;

class ShapeFlow : public MPxDeformerNode{
public:
    ShapeFlow()  {};
    virtual MStatus deform( MDataBlock& data, MItGeometry& itGeo, const MMatrix &localToWorldMatrix, unsigned int mIndex );
    static void*   creator();
    static MStatus initialize();
	static MTypeId id;
    static MString nodeName;
    static MObject aActive;
	static MObject aStartShape;
	static MObject aSlider;
	static MObject aDeltaTime;   // time step interval
	static MObject aShapeMatchingWeight;       // weight of "normal" shape matching
    int num; // number of points in the end shape
    MatrixXd current;  // current shape
private:
    Matrix3d rotationPart(const Matrix3d m);
};


MTypeId ShapeFlow::id( 0x00000010 );
MString ShapeFlow::nodeName( "shapeFlow" );
MObject ShapeFlow::aStartShape;
MObject ShapeFlow::aActive;
MObject ShapeFlow::aSlider;
MObject ShapeFlow::aDeltaTime;
MObject ShapeFlow::aShapeMatchingWeight;

void* ShapeFlow::creator() { return new ShapeFlow; }

// main
MStatus ShapeFlow::deform( MDataBlock& data, MItGeometry& itGeo, const MMatrix &localToWorldMatrix, unsigned int mIndex ) {
    MStatus status;
//    MThreadUtils::syncNumOpenMPThreads();    // for OpenMP
    
    // read end shape
    MObject oStartShape = data.inputValue( aStartShape ).asMesh();
    if ( oStartShape.isNull() )    {
        return MS::kSuccess;
    }
    MFnMesh fnEndShape( oStartShape, &status );
    CHECK_MSTATUS_AND_RETURN_IT( status );
    MPointArray endPoints, pts;
    fnEndShape.getPoints( endPoints );
    itGeo.allPositions(pts);
	MDataHandle hDeltaTime = data.inputValue( aDeltaTime );
    bool active = data.inputValue( aActive ).asBool();
	float delta = hDeltaTime.asFloat();
	float sm_weight = data.inputValue( aShapeMatchingWeight ).asFloat();
    int num = pts.length();
    // dummy attribute to force deform to be called
    MDataHandle hSlider = data.inputValue(aSlider);
    // if first time, load target shape
    if ( !active ){
        current = MatrixXd(3,num);
        for (int i = 0; i < num; i++) {
            current(0,i) = pts[i].x;
            current(1,i) = pts[i].y;
            current(2,i) = pts[i].z;
        }
        return MS::kSuccess;
    }
    // number of current and end points must be equal
    if (endPoints.length() != num) {
        return MS::kSuccess;
    }
    // load end shape
    MatrixXd end(3,num);
    for (int i = 0; i < num; i++) {
        end(0,i) = endPoints[i].x;
        end(1,i) = endPoints[i].y;
        end(2,i) = endPoints[i].z;
    }
    // compute next step
	MatrixXd Diff(3, num), Grad(3, num);
    Vector3d current_center = current.rowwise().mean();
    Vector3d end_center = end.rowwise().mean();
    
    // prepare moment matrix
    current.colwise() -= current_center;
    end.colwise() -= end_center;
    
	Matrix3d A, B, AB, ABB;
	A = current * end.transpose();
	B = (end * end.transpose()).inverse();
    AB = A * B;    // moment matrix ( minimizer of |AB P - Q|
    ABB = AB * B;
    Diff = rotationPart(AB) * end - current;
    
    // compute gradient
	Grad = end.norm() * current.norm() * ABB * (A.transpose() * ABB - Matrix3d::Identity()) * end;
    // update current position
    current += sm_weight * delta * Diff - delta * Grad;
    current.colwise() += current_center;
    /** FOR DEBUG: compute the energy
    * Matrix3d C =  B * A.transpose() * A * B;
    * float   energy = (C * C).trace() - 2 * C.trace() + 3;
    */
    // update points
    for (int i = 0; i < num; i++) {
        pts[i].x = current(0,i);
        pts[i].y = current(1,i);
        pts[i].z = current(2,i);
    }
    itGeo.setAllPositions(pts);

	return MS::kSuccess;
}

// Polar decomposition
Matrix3d ShapeFlow::rotationPart(const Matrix3d m){
    Matrix3d A= m*m.transpose();
	SelfAdjointEigenSolver<Matrix3d> eigensolver;
	eigensolver.computeDirect(A);
    Vector3d s = eigensolver.eigenvalues();
    Matrix3d U = Matrix3d(eigensolver.eigenvectors());
    s << sqrtf(s[0]), sqrtf(s[1]), sqrtf(s[2]);
    DiagonalMatrix<double,3> D(1.0f/s[0], 1.0f/s[1], 1.0f/s[2]);
    return m * U*D*U.transpose();
}

// setup attributes
MStatus ShapeFlow::initialize() {
    MFnTypedAttribute tAttr;
	MFnNumericAttribute nAttr;

	aStartShape = tAttr.create( "startShape", "ss", MFnData::kMesh );
    addAttribute( aStartShape );
    attributeAffects( aStartShape, outputGeom );
	aSlider = nAttr.create( "slider", "slider", MFnNumericData::kFloat, 0.0 );
    addAttribute( aSlider );
    attributeAffects( aSlider, outputGeom );
	aActive = nAttr.create( "active", "active", MFnNumericData::kBoolean, 0 );
    addAttribute( aActive );
    attributeAffects( aActive, outputGeom );
	aDeltaTime = nAttr.create( "delta", "delta", MFnNumericData::kFloat, 0.01 );
    addAttribute( aDeltaTime );
	aShapeMatchingWeight = nAttr.create( "shapeMatching", "smw", MFnNumericData::kFloat, 5.0 );
    addAttribute( aShapeMatchingWeight );

	return MS::kSuccess;
}

// (un)init plugin
MStatus initializePlugin( MObject obj ) {
    MStatus status;
    MFnPlugin plugin( obj, "CREST", "0.1", "Any");
    status = plugin.registerNode( ShapeFlow::nodeName, ShapeFlow::id, ShapeFlow::creator, ShapeFlow::initialize, MPxNode::kDeformerNode );
    CHECK_MSTATUS_AND_RETURN_IT( status );
    return status;
}
MStatus uninitializePlugin( MObject obj ) {
    MStatus   status;
    MFnPlugin plugin( obj );
    status = plugin.deregisterNode( ShapeFlow::id );
    CHECK_MSTATUS_AND_RETURN_IT( status );
    return status;
}

