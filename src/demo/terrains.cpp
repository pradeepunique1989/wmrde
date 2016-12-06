#include <wmrde/demo/terrains.h>

//generate a PlaneSurf object for flat ground, i.e. a level plane (z=0 everywhere)
void flat(SurfaceVector& surfaces) {
	Real pec[4]={0,0,1,0};
	surfaces.emplace_back(new PlaneSurf(pec));

}
void manyRamps(SurfaceVector& surfaces, const TerrainConfig& terrainInfo) {
    // std::vector<double> segmentLengths;
    // std::vector<double> segmentSlope;
    //
    // segmentLengths.push_back(2);
    // segmentSlope.push_back(0);
    //
    // segmentLengths.push_back(10);
    // // segmentSlope.push_back(10);
    // segmentSlope.push_back(10);
    //
    // segmentLengths.push_back(10);
    // // segmentSlope.push_back(-10);
    // segmentSlope.push_back(0);
    //
    // segmentLengths.push_back(10);
    // segmentSlope.push_back(-10);
    //
    // segmentLengths.push_back(8);
    // segmentSlope.push_back(0);


    double x = 0;
    double y = 0;
    double z = 0.01;
    const double segmentWidth = 10;

    surfaces.emplace_back(new TriMeshSurf());
    TriMeshSurf* surf_ptr = dynamic_cast<TriMeshSurf*>(surfaces.back().get());

    
    // Set the first two vertices (first edge)
    Vec3 v;
    setVec3(x,y+segmentWidth/2,z,v);
    surf_ptr->add_vertex(v);
    setVec3(x,y-segmentWidth/2,z,v);
    surf_ptr->add_vertex(v);

    int lastEdgeTopVertexIndex = 0;
    int lastEdgeBottomVertexIndex = 1;

    for (int i = 0; i < terrainInfo.numOfSegments ; i++)
    {
        const double nx = x + terrainInfo.segmentLength[i];
        const double ny = y;
        const double nz = z + std::tan(terrainInfo.segmentSlope[i] * M_PI/180.0f)*terrainInfo.segmentLength[i];
        setVec3(nx,ny+segmentWidth/2,nz,v);
        surf_ptr->add_vertex(v);
        setVec3(nx,ny-segmentWidth/2,nz,v);
        surf_ptr->add_vertex(v);

        const int edgeTopVertexIndex = lastEdgeBottomVertexIndex + 1;
        const int edgeBottomVertexIndex = edgeTopVertexIndex + 1;

        surf_ptr->add_quad(lastEdgeTopVertexIndex,lastEdgeBottomVertexIndex,edgeBottomVertexIndex,edgeTopVertexIndex);

        lastEdgeTopVertexIndex = edgeTopVertexIndex;
        lastEdgeBottomVertexIndex = edgeBottomVertexIndex;
        x = nx;
        y = ny;
        z = nz;
    }
}

//generate a TriMeshSurf object for a ramp
void ramp(SurfaceVector& surfaces) {

	//ramp surface
	Real w;		//width
	Real h;		//height
	Real a;		//angle
	Real lf;	//length of flat surface
	Real x,y;	//location of ramp center
	Real yaw;	//ramp orientation

	Real intom = .0254;
	if (0) {
		//Zoe rover ramp w/ flat
		w = 24*intom;
		h = 16*intom;
		a = asin(h/(27*intom));
		lf = 24*intom;
		x = 3;
		y = -.9;
		yaw = DEGTORAD(0);
	} else {
		//Zoe rover ramp w/ no flat
		w = 24*intom;
		h = 16.5*intom;
		a = asin(h/(30*intom));
		lf = 0;
		x = 3;
		y = -.9;
		yaw = DEGTORAD(0);
	}

	//do not change
	Real ls = h/tan(a); //length of sloped surface

	HomogeneousTransform HT;
	VecEuler euler;
	Vec3 pos; 
	
	setEuler(0,0,yaw,euler);
	setVec3(x,y,0,pos);
	eulerToRot(euler,HT);
	copyVec3(pos,HT+COL3);

	surfaces.emplace_back(new TriMeshSurf());
	TriMeshSurf* surf_ptr = dynamic_cast<TriMeshSurf*>(surfaces.back().get());
	
	if (lf > 0) {
		//ramp has a flat surface
		const int nv = 8;
		Vec3 V[nv];
		setVec3(ls+lf/2,  w/2, 0, V[0]);
		setVec3(ls+lf/2, -w/2, 0, V[1]);
		setVec3(lf/2,  w/2, h, V[2]);
		setVec3(lf/2, -w/2, h, V[3]);
		setVec3(-lf/2,  w/2, h, V[4]);
		setVec3(-lf/2, -w/2, h, V[5]);
		setVec3(-ls-lf/2,  w/2, 0, V[6]);
		setVec3(-ls-lf/2, -w/2, 0, V[7]);

		for (int i=0; i<nv; i++) {
			Vec3 v;
			applyHT(HT,V[i],v);
			surf_ptr->add_vertex(v);
		}
		surf_ptr->add_quad(0,2,3,1);
		surf_ptr->add_quad(2,4,5,3);
		surf_ptr->add_quad(4,6,7,5);

	} else {
		//no flat
		const int nv = 6;
		Vec3 V[nv];
		setVec3(ls, w/2, 0, V[0]);
		setVec3(ls, -w/2, 0, V[1]);
		setVec3(0, w/2, h, V[2]); 
		setVec3(0, -w/2, h, V[3]);
		setVec3(-ls, w/2, 0, V[4]);
		setVec3(-ls, -w/2, 0, V[5]);

		for (int i=0; i<nv; i++) {
			Vec3 v;
			applyHT(HT,V[i],v);
			surf_ptr->add_vertex(v);
		}
		surf_ptr->add_quad(0,2,3,1);
		surf_ptr->add_quad(2,4,5,3);
	}	

}

//generate a GridSurf object, read in data from a file
void grid(SurfaceVector& surfaces, const std::string FileName) {

	surfaces.emplace_back(new GridSurf());

	//cast to derived class, call readfile member function
	dynamic_cast<GridSurf*>(surfaces.back().get())->readfile( FileName );
}
