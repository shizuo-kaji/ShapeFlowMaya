/**
 * @file shapeMatching.cpp
 * @brief ShapeMatching plugin for Maya
 * @section LICENSE The MIT License
 * @section  requirements:  Eigen library, Maya
 * @section Limitation: the shapes must be connected
 * @version 0.10
 * @date  1/Nov/2013
 * @author Shizuo KAJI
 */

#pragma comment(linker, "/export:initializePlugin /export:uninitializePlugin")

#include "StdAfx.h"

#include <maya/MFnPlugin.h>

#include <numeric>
#include <Eigen/Dense>


using namespace std;
using namespace Eigen;

class ShapeMatching : public MPxDeformerNode
{
public:
    ShapeMatching() {};
    virtual MStatus deform( MDataBlock& data, MItGeometry& itGeo, const MMatrix &localToWorldMatrix, unsigned int mIndex );
    static  void*   creator();
    static  MStatus initialize();
	static MTypeId id;
    static MString nodeName;
	static MObject aStartShape;
    static MObject aActive;
	static MObject aSlider;
	static MObject aDeltaTime;
	static MObject aStiffness;
	static MObject aAttenuation;
private:
    MatrixXd current, velocity;
    Matrix3d rotationPart(const Matrix3d m);
};


MTypeId ShapeMatching::id( 0x00000020 );
MString ShapeMatching::nodeName( "shapeMatching" );
MObject ShapeMatching::aStartShape;
MObject ShapeMatching::aSlider;
MObject ShapeMatching::aActive;
MObject ShapeMatching::aDeltaTime;
MObject ShapeMatching::aStiffness;
MObject ShapeMatching::aAttenuation;

void* ShapeMatching::creator() { return new ShapeMatching; }

// Compute
MStatus ShapeMatching::deform( MDataBlock& data, MItGeometry& itGeo, const MMatrix &localToWorldMatrix, unsigned int mIndex ){
    MStatus status;
//    MThreadUtils::syncNumOpenMPThreads();    // for OpenMP
    
    // read start shape
    MObject oStartShape = data.inputValue( aStartShape ).asMesh();
    if ( oStartShape.isNull() )    {
        return MS::kSuccess;
    }
    MFnMesh fnStartShape( oStartShape, &status );
    CHECK_MSTATUS_AND_RETURN_IT( status );
    MPointArray endPoints, pts;
    itGeo.allPositions(pts);
    fnStartShape.getPoints( endPoints );
    // read attributes
    MDataHandle hSlider = data.inputValue( aSlider );
    bool active = data.inputValue( aActive ).asBool();
    float delta = data.inputValue( aDeltaTime ).asFloat();
    float stiffness = data.inputValue( aStiffness ).asFloat();
    float attenution = data.inputValue( aAttenuation ).asFloat();
    int num = pts.length();
    // first-time setup
    if (!active){
		current = MatrixXd(3,num);
		velocity = MatrixXd::Zero(3,num);
        for (int i = 0; i < num; i++) {
            current(0,i) = pts[i].x;
            current(1,i) = pts[i].y;
            current(2,i) = pts[i].z;
        }
        return MS::kSuccess;
    }
    // number of current and end points must be equal
    if (endPoints.length() != num){
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
    Vector3d current_center = current.rowwise().mean();
    Vector3d end_center = end.rowwise().mean();
    
    // prepare moment matrix
    current.colwise() -= current_center;
    end.colwise() -= end_center;
    Matrix3d moment = end * current.transpose();
	// Update current vertices and velocity of vertices
    velocity += delta * stiffness * (rotationPart(moment) * end - current);
    current += delta * velocity;
    velocity *= attenution;
    current.colwise() += current_center;

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
Matrix3d ShapeMatching::rotationPart(const Matrix3d m){
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
MStatus ShapeMatching::initialize(){
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
	aStiffness = nAttr.create( "stiffness", "stf", MFnNumericData::kFloat, 1.0 );
    addAttribute( aStiffness );
	aAttenuation = nAttr.create( "attenuation", "att", MFnNumericData::kFloat, 0.9 );
    addAttribute( aAttenuation );

	return MS::kSuccess;
}




// (un)init plugin
MStatus initializePlugin( MObject obj ){
    MStatus status;
    MFnPlugin plugin( obj, "CREST", "0.1", "Any");
    status = plugin.registerNode( ShapeMatching::nodeName, ShapeMatching::id, ShapeMatching::creator, ShapeMatching::initialize, MPxNode::kDeformerNode );
    CHECK_MSTATUS_AND_RETURN_IT( status );
    return status;
}
MStatus uninitializePlugin( MObject obj ){
    MStatus   status;
    MFnPlugin plugin( obj );
    status = plugin.deregisterNode( ShapeMatching::id );
    CHECK_MSTATUS_AND_RETURN_IT( status );
    return status;
}

