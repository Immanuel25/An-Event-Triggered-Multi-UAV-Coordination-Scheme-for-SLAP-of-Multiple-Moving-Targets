function GTF_PlotVessel(pos, psi, scale_num, time, fuselage_colour,alpha)
% =========================================================================
% Go To Formation - Plot AUV.
% =========================================================================
% Plots a 3D version of a torpedo-shaped AUV. Uses Bruce R. Land's
% Hierarchical Matlab Renderer.
% http://www.nbb.cornell.edu/neurobio/land/PROJECTS/Hierarchy/
%
% Code by: Andreas J. Haeusler
% Last Revision: October 2009
%
%
% 2009-10-07    clean new implementation
%==========================================================================

% Example position updates: x += 1, z -= 1
position(1) = 1;  % Increment x by 1
position(3) = -1;  % Decrement z by 1

% Build the fuselage
tmpsphere = UnitSphere(2);
tmpsphere.facecolor = fuselage_colour;
tailcap = translate(scale(tmpsphere, 3, 1.5, 1), 1 + position(1), 0, 1.01 + position(3));
nose = translate(scale(tmpsphere, 2, 1.5, 1), -1 + position(1), 0, 1.01 + position(3));
cyl1 = UnitCylinder(4);
cyl1 = translate(rotateY(scale(cyl1, 1, 1.5, 1), 90), position(1), 0, 1.01 + position(3));
cyl1.facecolor = fuselage_colour;
fuselage = combine(cyl1, nose, tailcap);


tmpcube = UnitCube;
cockpit = translate(scale(tmpcube, 1, 0.7, 0.5), -.5 + position(1), 0, 2 + position(3));
tmpcyl = UnitCylinder(4);
tmpsphere = UnitSphere(2);
tmpcyl.facecolor = fuselage_colour;
flagpole = translate(scale(tmpcyl, .05, .05, .5), -.3 + position(1), 0, 3 + position(3));
flag = translate(scale(tmpcube, .05, .5, .2), -.3 + position(1), 0, 3.3 + position(3));
fishingpole = translate(scale(tmpsphere, .2, .2, 1.5), 1 + position(1), 0, 5 + position(3));
fishingpole = rotateY(fishingpole, 45);
cockpit = combine(cockpit, flagpole, flag, fishingpole);



vessel = combine(fuselage,cockpit);
% the following order must be kept as it is 
% first rotate, then scale, then translate
vessel = rotateZ(vessel, psi);   
vessel = scale(vessel, scale_num, scale_num, scale_num);
vessel = translate(vessel, pos(1), pos(2), pos(3));

% clf
renderpatch(vessel,alpha,fuselage_colour);
% light('position',[0,0,10]) ;
daspect([1 1 1])

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%% The following code is copied from %%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%  Hierarchical Matlab Renderer %%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function count = renderwire(objIn,alpha)
  %hierarchical render function for structs and cell arrays
  %as a wire frame image.
  %Takes either a cell array or a single struct as input.
  %For each struct, can set:
  %  maps facecolor to edgecolor
  %  edgecolor: default='cyan'
  %  visible: default='on'

  if (iscell(objIn)) %a list of structs

     for i=1:length(objIn)

        obj=patch(objIn{i},'FaceAlpha', alpha);

        if (isfield(objIn{i},'facecolor'))
           ecolor=objIn{i}.facecolor;
        else
           ecolor=[0,1,1];
        end

        if (isfield(objIn{i},'edgecolor'))
           ecolor=objIn{i}.edgecolor;
        end

        if (isfield(objIn{i},'visible'))
           vis=objIn{i}.visible;
        else
           vis='on';
        end

        set(obj, 'FaceColor', 'none', ...
                 'EdgeColor', ecolor, ...
                 'Visible',vis);
     end  
     count=i;

   elseif (isstruct(objIn)) %must be a single struct   
      obj=patch(objIn,'FaceAlpha', alpha);

      if (isfield(objIn,'facecolor'))
           ecolor=objIn.facecolor;
        else
           ecolor=[0,1,1];
        end

        if (isfield(objIn,'edgecolor'))
           ecolor=objIn.edgecolor;
        end


        if (isfield(objIn,'visible'))
           vis=objIn.visible;
        else
           vis='on';
        end

        set(obj, 'FaceColor', 'none', ...
                 'EdgeColor', ecolor, ...
                 'Visible',vis);
        count=1;   
   end %if   
end

function count = renderpatch(objIn,alpha,fuselage_colour)
  %hierarchical render function for structs and cell arrays
  %Takes either a cell array or a single struct as input.
  %For each struct, can set:
  %  facecolor: default=cyan
  %  edgecolor: default='none'
  %  ambientstrength: default=.6
  %  specularstrength: default=.2
  %  specularexponent: default=10
  %  facelighting: default='phong'
  %  diffusestrength: default=.5
  %  visible: default='on'

  if (iscell(objIn)) %a list of structs

     for i=1:length(objIn)

        obj=patch(objIn{i},'FaceAlpha', alpha);

        if (isfield(objIn{i},'facecolor'))
           fcolor=objIn{i}.facecolor;
        else
           fcolor=[0,1,1];
        end

        if (isfield(objIn{i},'edgecolor'))
           ecolor=objIn{i}.edgecolor;
        else
%            ecolor='none';
           ecolor=fuselage_colour;
           
        end

        if (isfield(objIn{i},'ambientstrength'))
           ambstr=objIn{i}.ambientstrength;
        else
           ambstr=.6;
        end

        if (isfield(objIn{i},'specularstrength'))
           spcstr=objIn{i}.specularstrength;
        else
           spcstr=.2;
        end

        if (isfield(objIn{i},'specularexponent'))
           spcexp=objIn{i}.specularexponent;
        else
           spcexp=10;
        end

        if (isfield(objIn{i},'facelighting'))
           facelight=objIn{i}.facelighting;
        else
           facelight='phong';
        end

        if (isfield(objIn{i},'diffusestrength'))
           difstr=objIn{i}.diffusestrength;
        else
           difstr=.5;
        end

        if (isfield(objIn{i},'visible'))
           vis=objIn{i}.visible;
        else
           vis='on';
        end


        set(obj, 'FaceColor', fcolor, ...
                 'EdgeColor', ecolor, ...
                 'AmbientStrength',ambstr,...
                 'SpecularStrength',spcstr, ...
                 'SpecularExponent', spcexp, ...
                 'FaceLighting', facelight, ...
                 'DiffuseStrength', difstr, ...
                 'Visible',vis);
     end  
     count=i;

   elseif (isstruct(objIn)) %must be a single struct   
      obj=patch(objIn,'FaceAlpha', alpha);

      if (isfield(objIn,'facecolor'))
           fcolor=objIn.facecolor;
        else
           fcolor=[0,1,1];
        end

        if (isfield(objIn,'edgecolor'))
           ecolor=objIn.edgecolor;
        else
           ecolor='none';
        end

        if (isfield(objIn,'ambientstrength'))
           ambstr=objIn.ambientstrength;
        else
           ambstr=.6;
        end

        if (isfield(objIn,'specularstrength'))
           spcstr=objIn.specularstrength;
        else
           spcstr=.2;
        end

        if (isfield(objIn,'specularexponent'))
           spcexp=objIn.specularexponent;
        else
           spcexp=10;
        end

        if (isfield(objIn,'facelighting'))
           facelight=objIn.facelighting;
        else
           facelight='phong';
        end

        if (isfield(objIn,'diffusestrength'))
           difstr=objIn.diffusestrength;
        else
           difstr=.5;
        end

        if (isfield(objIn,'visible'))
           vis=objIn.visible;
        else
           vis='on';
        end

        set(obj, 'FaceColor', fcolor, ...
                 'EdgeColor', ecolor, ...
                 'AmbientStrength',ambstr,...
                 'SpecularStrength',spcstr, ...
                 'SpecularExponent', spcexp, ...
                 'FaceLighting', facelight, ...
                 'DiffuseStrength', difstr, ...
                 'Visible',vis);
        count=1;   
   end %if   
end

function objOut = combine(varargin)

  %Takes a list of opjects (structs and cell arrays) and
  %returns a cell array

  num=length(varargin);

  if (num==0)
     error('must have at least one input object');
  end

  objOut={};

  for i=1:num
     if (iscell(varargin{i})) %a list of structs
        objOut=[objOut, varargin{i}];        
     elseif (isstruct(varargin{i})) %must be a single struct         
        objOut=[objOut, {varargin{i}}];    
     else
        error('input must be s struct or cell array')
     end %if (iscell(varargin(i)))   
  end %for
end

function polyhedron=Polyhedra(ptype)

%one of five regular polyhedra
%in a format consistent with hieracrhical 
%modeler scaled to fit inside a unit sphere

if (strcmp(ptype,'tetrahedron'))
   polyhedron.vertices=[...
         0.453608  0.890430  0.037037
      0.544331 -0.628540 -0.555555
      -0.090722 -0.366647  0.925925
      -0.907218  0.104757 -0.407407
   ];
   
   polyhedron.faces=1+[...
         0 1 2
      0 1 3
      0 2 3
      1 2 3
   ];
   
elseif (strcmp(ptype,'cube'))
   polyhedron.vertices=0.5774*[...
         0 0 0; 1 0 0; 1 1 0; 0 1 0; ...
         0 0 1; 1 0 1; 1 1 1; 0 1 1;] ;
   
   polyhedron.vertices=polyhedron.vertices *2 - 0.5774;
   
   polyhedron.faces=[ 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8; ...
         1 2 3 4; 5 6 7 8; ] ;
   
elseif (strcmp(ptype,'octahedron'))
   polyhedron.vertices=[...
         0.408248  0.707107 -0.577350
      0.408248 -0.707107 -0.577350
      0.816496  0.000000  0.577350
      -0.408248  0.707107  0.577350
      -0.408248 -0.707107  0.577350
      -0.816496  0.000000 -0.577350
   ];
   polyhedron.faces=1+[...
         3 2 0
      3 0 5
      5 0 1
      0 2 1
      3 5 4
      5 1 4
      4 1 2
      4 2 3
   ];
   
elseif (strcmp(ptype,'dodecahedron'))
   polyhedron.vertices=[...
         0.175988  0.745447 -0.642912
      0.230032  0.972675  0.031428
      0.276500 -0.379561 -0.882882
      0.392664 -0.847627 -0.356851
      0.417990  0.215329  0.882563
      0.480109 -0.479965  0.734253
      0.643598  0.214231 -0.734769
      0.731044  0.581893  0.356335
      0.831555 -0.543116  0.116366
      0.986641  0.113149 -0.117200
      -0.175988 -0.745447  0.642912
      -0.230033 -0.972675 -0.031428
      -0.276499  0.379562  0.882882
      -0.392664  0.847627  0.356851
      -0.417989 -0.215329 -0.882563
      -0.480109  0.479965 -0.734254
      -0.643598 -0.214231  0.734769
      -0.731044 -0.581893 -0.356336
      -0.831555  0.543115 -0.116366
      -0.986641 -0.113149  0.117200
   ];
   polyhedron.faces=1+[...
         17 19 18 
      18 15 14 
      17 18 14 
      1 0 15 
      15 18 13 
      1 15 13 
      6 2 14 
      14 15 0 
      6 14 0 
      3 11 17 
      17 14 2 
      3 17 2 
      10 16 19 
      19 17 11 
      10 19 11 
      12 13 18 
      18 19 16 
      12 18 16 
      16 10 5 
      5 4 12 
      16 5 12 
      11 3 8 
      8 5 10 
      11 8 10 
      2 6 9 
      9 8 3 
      2 9 3 
      0 1 7 
      7 9 6 
      0 7 6 
      13 12 4 
      4 7 1 
      13 4 1 
      7 4 5 
      5 8 9 
      7 5 9 
   ];
   
elseif (strcmp(ptype,'icosahedron'))
   polyhedron.vertices=[...
         0.250254 -0.916162 -0.313083
      0.320072 -0.078054 -0.944171
      0.437594 -0.598061  0.671442
      0.550563  0.758025 -0.349682
      0.623196  0.436642  0.648821
      0.975676 -0.177816 -0.128204
      -0.250253  0.916161  0.313082
      -0.320073  0.078054  0.944172
      -0.437593  0.598061 -0.671442
      -0.550563 -0.758024  0.349683
      -0.623195 -0.436643 -0.648822
      -0.975676  0.177817  0.128204
   ];
   polyhedron.faces=1+[...
         3 4 5
      4 7 2
      3 5 1
      5 2 0
      3 1 8
      1 0 10
      3 8 6
      8 10 11
      3 6 4
      6 11 7
      2 5 4
      9 2 7
      0 1 5
      9 0 2
      10 8 1
      9 10 0
      11 6 8
      9 11 10
      7 4 6
      9 7 11
   ];
   
else
   error('type must be:tetrahedron, cube, octahedron,dodecahedron,icosahedron');
   
end
end

function objOut = rotateX(objIn,a)
  %hierarchical rotate function for structs and cell arrays
  a=a/57.29;  %degrees to radians
  if (iscell(objIn)) %a list of structs
   for i=1:length(objIn)
      objOut{i}=objIn{i};
      V=objOut{i}.vertices;
      V=[V(:,1), ...
            cos(a)*V(:,2)-sin(a)*V(:,3), ...
            sin(a)*V(:,2)+cos(a)*V(:,3)];
      objOut{i}.vertices=V;   
   end      
  elseif (isstruct(objIn)) %must be a single struct   
    V=objIn.vertices;
    V=[V(:,1), ...
            cos(a)*V(:,2)-sin(a)*V(:,3), ...
            sin(a)*V(:,2)+cos(a)*V(:,3)];
    objOut=objIn;
    objOut.vertices=V; 
  else
    error('input must be s struct or cell array')
  end %if   
end

function objOut = rotateY(objIn,a)
  %hierarchical rotate function for structs and cell arrays
  a=a/57.29;  %degrees to radians
  if (iscell(objIn)) %a list of structs
   for i=1:length(objIn)
      objOut{i}=objIn{i};
      V=objOut{i}.vertices;

      V=[cos(a)*V(:,1)+sin(a)*V(:,3), ...
            V(:,2), ...
            -sin(a)*V(:,1)+cos(a)*V(:,3)];

      objOut{i}.vertices=V;   
   end      
  elseif (isstruct(objIn)) %must be a single struct   
    V=objIn.vertices;

    V=[cos(a)*V(:,1)+sin(a)*V(:,3), ...
            V(:,2), ...
            -sin(a)*V(:,1)+cos(a)*V(:,3)];

    objOut=objIn;
    objOut.vertices=V; 
  else
    error('input must be s struct or cell array')
  end %if   
end

function objOut = rotateZ(objIn,a)
  %hierarchical rotate function for structs and cell arrays
  a=a/57.29;  %degrees to radians
  if (iscell(objIn)) %a list of structs
   for i=1:length(objIn)
      objOut{i}=objIn{i};
      V=objOut{i}.vertices;

      V=[cos(a)*V(:,1)-sin(a)*V(:,2), ...
            sin(a)*V(:,1)+cos(a)*V(:,2), ...
            V(:,3)];

      objOut{i}.vertices=V;   
   end      
  elseif (isstruct(objIn)) %must be a single struct   
    V=objIn.vertices;

    V=[cos(a)*V(:,1)-sin(a)*V(:,2), ...
            sin(a)*V(:,1)+cos(a)*V(:,2), ...
            V(:,3)];

    objOut=objIn;
    objOut.vertices=V; 
  else
    error('input must be s struct or cell array')
  end %if   
end

function objOut = scale(objIn,x,y,z)
  %hierarchical scale function for structs and cell arrays

  if (iscell(objIn)) %a list of structs
   for i=1:length(objIn)
      objOut{i}=objIn{i};
      V=objIn{i}.vertices;
      V=[V(:,1)*x, V(:,2)*y, V(:,3)*z];
      objOut{i}.vertices=V;
   end      
  elseif (isstruct(objIn)) %must be a single struct   
    V=objIn.vertices;
    V=[V(:,1)*x, V(:,2)*y, V(:,3)*z];
    objOut=objIn;
    objOut.vertices=V; 
  else
    error('input must be s struct or cell array')
  end %if   
end

function objOut = translate(objIn,x,y,z)
  %hierarchical translate function for structs and cell arrays
  %Input is:
  %  an struct consisting of a vertex and face array
  %  or an cell array of structs
  %  an x,y,z translation
  if (iscell(objIn)) %a list of structs
   for i=1:length(objIn)
      objOut{i}=objIn{i};
      V=objOut{i}.vertices;
      V=[V(:,1)+x, V(:,2)+y, V(:,3)+z];
      objOut{i}.vertices=V;   
   end      
  elseif (isstruct(objIn)) %must be a single struct   
    V=objIn.vertices;
    V=[V(:,1)+x, V(:,2)+y, V(:,3)+z];
    objOut=objIn;
    objOut.vertices=V; 
  else
    error('input must be s struct or cell array')
  end %if   
end

function cube=UnitCube
  %unit cube in a format consistent with hieracrhical 
  %modeler

  %Define a cube
  cube.vertices=[ 0 0 0; 1 0 0; 1 1 0; 0 1 0; ...
        0 0 1; 1 0 1; 1 1 1; 0 1 1;] ;
  cube.faces=[ 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8; ...
        1 2 3 4; 5 6 7 8; ] ;

  cube.vertices = cube.vertices * 2 - 1;
end

function cylinder=UnitCylinder(res)
coder.extrinsic('isosurface');
  %unit sphere in a format consistent with hieracrhical 
  %modeler
  %The input paramenter is related to the sphere resolution.
  %Range 1-10. Higher number is better approximation
  %1=> 4-sided tube
  %1.5=> 8-sided tube
  %2=> 48 faces
  %3=> 80 faces
  %5=>136 faces
  %10=>272 faces

  %range check
  if (res>10)
     res=10;
  elseif (res<1)
     res=1;
  end

  res=1/res;
  [x,y,z]=meshgrid(-1-res:res:1+res, ...
     -1-res:res:1+res, -1:1:1);
  w=sqrt(x.^2+y.^2);
  cylinder=isosurface(x,y,z,w,1);

end

function sphere=UnitSphere(res)
coder.extrinsic('isosurface');
  %unit sphere in a format consistent with hieracrhical 
  %modeler
  %The input paramenter is related to the sphere resolution.
  %Range 1-10. Higher number is better approximation
  %1=>octahedron
  %1.5=> 44 faces
  %2=> 100 faces
  %2.5 => 188 faces
  %3=> 296 faces
  %5=> 900 faces
  %10=>3600 faces

  %range check
  if (res>10)
     res=10;
  elseif (res<1)
     res=1;
  end

  res=1/res;
  [x,y,z]=meshgrid(-1-res:res:1+res, ...
     -1-res:res:1+res, -1-res:res:1+res);
  w=sqrt(x.^2+y.^2+z.^2);
  sphere=isosurface(x,y,z,w,1);

end

function square=UnitSquare
  %unit square in the x-y plane
  %in a format consistent with hieracrhical 
  %modeler

   %Define a square
  square.vertices= ...
     [-1, -1, 0;
      -1, 1, 0;
       1, 1 ,0;
       1, -1, 0];
  square.faces=[1 2 3 4];
end

function surface=UnitSurface(res)
  %unit flat surface in a format consistent with hieracrhical 
  %modeler
  %The input paramenter is related to the sphere resolution.
  %Range 1-10. Higher number is better approximation
  %1=> 8 triangular faces
  %2=> 32 faces
  %5=>200 faces
  %10=>800 faces
  %20=>3200 faces
  %50=>20000 faces

  %range check
  if (res>100)
     res=100;
  elseif (res<1)
     res=1;
  end

  res=1/res;
  [x,y,z]=meshgrid(-1:res:1, ...
     -1:res:1, -1:1:1);
  w=z;
  surface=isosurface(x,y,z,w,0);

end

function torus=UnitTorus(radius, res)
  %unit torus in a format consistent with hieracrhical 
  %modeler
  %The first parameter is the radius of the cross-section
  %The second input paramenter is related to the  resolution.
  %Range 1-10. Higher number is better approximation
  %Res input of less than 3 makes a very rough torus
  %3=> 384 faces
  %5=>1230 faces
  %10=>5100 faces

  %range check
  if (res>10)
     res=10;
  elseif (res<1)
     res=1;
  end

  res=1/res;
  [x,y,z]=meshgrid(-1-radius-res:res:1+radius+res, ...
     -1-radius-res:res:1+radius+res, -radius-res:res:radius+res);

  w=(1-sqrt(x.^2+y.^2)).^2 +z.^2 - radius^2;

  torus=isosurface(x,y,z,w,0);

end
