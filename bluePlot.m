function  graphRobot=bluePlot(x,y,z,phi,psi,scaleRobot)
load 'blueSTL.mat'

%Matrix Rotation z axis
Rz=[ cos(phi) -sin(phi) 0; sin(phi) cos(phi) 0; 0 0 1];
%%llantas delanteras transformada
Rzi=[ cos(phi+psi) -sin(phi+psi) 0; sin(phi+psi) cos(phi+psi) 0; 0 0 1];
dyi=1.1*sin(phi);
dxi=1.1*cos(phi);

Rzd=[ cos(phi+psi) -sin(phi+psi) 0; sin(phi+psi) cos(phi+psi) 0; 0 0 1];
dyd=1.1*sin(phi);
dxd=1.1*cos(phi);

Rzeje=[ cos(phi+psi) -sin(phi+psi) 0; sin(phi+psi) cos(phi+psi) 0; 0 0 1];
dye=1.1*sin(phi);
dxe=1.1*cos(phi);

robotPatch = Rz*base.vertices';
robotPatch(1,:)=robotPatch(1,:)*scaleRobot+x; 
robotPatch(2,:)=robotPatch(2,:)*scaleRobot+y;
robotPatch(3,:)=robotPatch(3,:)*scaleRobot+z;

graphRobot(1) = patch('Faces',base.faces,'Vertices',robotPatch','FaceColor',[0,0,0.7],'EdgeColor','none');

robotPatch = Rz*eje_trasero.vertices';
robotPatch(1,:)=robotPatch(1,:)*scaleRobot+x; 
robotPatch(2,:)=robotPatch(2,:)*scaleRobot+y;
robotPatch(3,:)=robotPatch(3,:)*scaleRobot+z;


graphRobot(2) = patch('Faces',eje_trasero.faces,'Vertices',robotPatch','FaceColor',[0.8,0.8,0.8],'EdgeColor','none');

robotPatch = Rz*r_trasera.vertices';
robotPatch(1,:)=robotPatch(1,:)*scaleRobot+x; 
robotPatch(2,:)=robotPatch(2,:)*scaleRobot+y;
robotPatch(3,:)=robotPatch(3,:)*scaleRobot+z;


graphRobot(3) = patch('Faces',r_trasera.faces,'Vertices',robotPatch','FaceColor','k','EdgeColor','none');

robotPatch = Rz*velodyne.vertices';
robotPatch(1,:)=robotPatch(1,:)*scaleRobot+x; 
robotPatch(2,:)=robotPatch(2,:)*scaleRobot+y;
robotPatch(3,:)=robotPatch(3,:)*scaleRobot+z;


graphRobot(4) = patch('Faces',velodyne.faces,'Vertices',robotPatch','FaceColor',[0.8,0.8,0.8],'EdgeColor','none');

robotPatch = Rzi*r_delantera_i.vertices';
robotPatch(1,:)=robotPatch(1,:)*scaleRobot+x+dxi; 
robotPatch(2,:)=robotPatch(2,:)*scaleRobot+y+dyi;
robotPatch(3,:)=robotPatch(3,:)*scaleRobot+z;

graphRobot(5) = patch('Faces',r_delantera_i.faces,'Vertices',robotPatch','FaceColor','k','EdgeColor','none');

robotPatch = Rzd*r_delantera_d.vertices';
robotPatch(1,:)=robotPatch(1,:)*scaleRobot+x+dxd; 
robotPatch(2,:)=robotPatch(2,:)*scaleRobot+y+dyd;
robotPatch(3,:)=robotPatch(3,:)*scaleRobot+z;

graphRobot(6) = patch('Faces',r_delantera_d.faces,'Vertices',robotPatch','FaceColor','k','EdgeColor','none');

robotPatch = Rzeje*eje_delantero.vertices';
robotPatch(1,:)=robotPatch(1,:)*scaleRobot+x+dxe; 
robotPatch(2,:)=robotPatch(2,:)*scaleRobot+y+dye;
robotPatch(3,:)=robotPatch(3,:)*scaleRobot+z;


graphRobot(7) = patch('Faces',eje_delantero.faces,'Vertices',robotPatch','FaceColor',[0.8,0.8,0.8],'EdgeColor','none');

