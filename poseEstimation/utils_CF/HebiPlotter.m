classdef HebiPlotter < handle
    % HebiPlotter visualize realistic looking HEBI modules
    %
    %   Currently only the HEBI elbow joints (snake links) and pipe
    %   joints can be plotted
    %
    %   HebiPlotter Methods (constructor):
    %      HebiPlotter  - constructor
    %
    %   HebiPlotter Methods:
    %      plot         - plots the robot in the specified configuation
    %      setBaseFrame - sets the frame of the first link
    %
    %   Examples:
    %      plt = HebiPlotter();
    %      plt.plot([.1,.1]);
    %
    %      plt = HebiPlotter(16, 'resolution', 'high');
    %      plt.plot(zeros(16,1));
    
    methods(Access = public)
        %Constructor
        function this = HebiPlotter(varargin)
        %HEBIPLOTTER
        %Arguments:
        %
        %Optional Parameters:
        %  'resolution'        - 'low' (default), 'high' 
        %  'lighting'          - 'on' (default), 'off'
        %  'frame'             - 'base' (default), 'VC', 'gravity', 'head'
        %  'JointTypes'        - cell array of joint types
        %
        %Examples:
        %  plt = HebiPlotter()
        %  plt = HebiPlotter('resolution', 'high')
        %
        %  links = {{'FieldableElbowJoint'},
        %           {'FieldableStraightLink', 'ext1', .1, 'twist', 0},
        %           {'Foot', 'ext1', .1, 'twist', 0}};
        %  plt = HebiPlotter('JointTypes', links)  


            p = inputParser;
            expectedResolutions = {'low', 'high'};
            expectedLighting = {'on','off', 'far'};
            expectedFrames = {'Base', 'VC', 'gravity','head'};
            
            % addRequired(p, 'numLinks', @isnumeric);
            addParameter(p, 'resolution', 'low', ...
                         @(x) any(validatestring(x, ...
                                                 expectedResolutions)));
            addParameter(p, 'frame', 'Base',...
                         @(x) any(validatestring(x, expectedFrames)));

            addParameter(p, 'lighting', 'on',...
                         @(x) any(validatestring(x, ...
                                                 expectedLighting)));
            addParameter(p, 'JointTypes', {});
            addParameter(p, 'drawWhen', 'now');
            addParameter(p, 'accelOffsets', []);     
            addParameter(p, 'gyroOffsets', []);   
            addParameter(p, 'gyrosTrustability', []);
            addParameter(p, 'accTrustability', []);


            parse(p, varargin{:});

            this.lowResolution = strcmpi(p.Results.resolution, 'low');

            this.firstRun = true;
            this.firstPlot = true;
            
            this.lighting = p.Results.lighting;
            this.setKinematicsFromJointTypes(p.Results.JointTypes);
            this.frameType = p.Results.frame;
            this.drawNow = strcmp(p.Results.drawWhen, 'now');
            this.accelOffsets = p.Results.accelOffsets;
            this.gyroOffsets = p.Results.gyroOffsets;
            this.gyrosTrustability = p.Results.gyrosTrustability;
            this.accTrustability = p.Results.accTrustability;
            
            
            if (strcmp(this.frameType, 'gravity')) 
                this.readyToPlot = false;
            else
                this.readyToPlot = true;
            end
           
                
        end
        
        function [fk, fk_out] = plot_with_head(this, anglesOrFbk, T_head, accelerations)
        % PLOT plots the robot in the configuration specified by
        % angles
            this.Thead = T_head;
                       
            if (isnumeric(anglesOrFbk))
                angles = anglesOrFbk;
                fbk = [];
                if (strcmpi(this.frameType, 'gravity'))
                    error(['Input needs to be a feedback  '...
                        '(you choose gravity frame)']);
                end
                if ~isempty(accelerations)
                    fbk.accelX = accelerations(1,:);
                    fbk.accelY = accelerations(2,:);
                    fbk.accelZ = accelerations(3,:);

                end
            else
                try
                    angles = anglesOrFbk.position;
                    fbk = anglesOrFbk;
                catch
                    error(['Input needs to be either a list of angles ' ...
                           'or feedback']);
                end
            end

            fk = [];fk_out=[];
            
            if(this.firstRun)
                if (strcmp(this.frameType, 'gravity'))
                    snakeData = setupSnakeData( 'SEA Snake', length(angles));
%                     this.gyrosTrustability
                    this.CF = ComplementaryFilter(snakeData, 'accelOffsets', this.accelOffsets, ...
                        'gyroOffsets', this.gyroOffsets, ...
                        'gyrosTrustability', this.gyrosTrustability);
%                         'accTrustability', this.accTrustability);
%                     this.readyToPlot = this.CF.calibrateGyros(fbk);
                end
                this.firstRun = false;
            end;
            
%             if (~this.readyToPlot)
%                 disp('Calibrating gyros...');
%                 this.readyToPlot = this.CF.calibrateGyros(fbk);
%                 if (this.readyToPlot)
%                     disp('Gyros calibrated!');
% %                     pause();
%                 end
%             end
                
                if this.firstPlot
                    [fk, fk_out] = initialPlot(this, angles, fbk);
                    this.firstPlot = false;
                else
                    [fk, fk_out] = updatePlot(this, angles, fbk);
                end

            
            if(this.drawNow)
                drawnow
            end
            
%             this.readyToPlot 
%             this.firstPlot
        end
        
        function [fk, fk_out] = plot(this, anglesOrFbk)
        % PLOT plots the robot in the configuration specified by
        % angles
                       
            if (isnumeric(anglesOrFbk))
                angles = anglesOrFbk;
                fbk = [];
                if (strcmpi(this.frameType, 'gravity'))
                    error(['Input needs to be a feedback  '...
                        '(you choose gravity frame)']);
                end
            else
                try
                    angles = anglesOrFbk.position;
                    fbk = anglesOrFbk;
                catch
                    error(['Input needs to be either a list of angles ' ...
                           'or feedback']);
                end
            end

            fk = [];fk_out=[];
            
            if(this.firstRun)
                if (strcmp(this.frameType, 'gravity'))
                    snakeData = setupSnakeData( 'SEA Snake', length(angles));
                    this.CF = ComplementaryFilter(snakeData, 'accelOffsets', this.accelOffsets, ...
                        'gyroOffsets', this.gyroOffsets, ...
                        'gyrosTrustability', this.gyrosTrustability, ...
                        'accTrustability', this.accTrustability);
                    %                     this.readyToPlot = this.CF.calibrateGyros(fbk);
                end
                this.firstRun = false;
            end;
            
%             if (~this.readyToPlot)
%                 disp('Calibrating gyros...');
%                 this.readyToPlot = this.CF.calibrateGyros(fbk);
%                 if (this.readyToPlot)
%                     disp('Gyros calibrated!');
% %                     pause();
%                 end
%             end
                
                if this.firstPlot
                    initialPlot(this, angles, fbk);
                    this.firstPlot = false;
                else
                    [fk, fk_out] = updatePlot(this, angles, fbk);
                end
            
            if(this.drawNow)
                drawnow
            end
        end
        
        function setBaseFrame(this, frame)
            %SETBASEFRAME sets the frame of the first link in the kinematics
            %chain
            %
            % Arguments:
            % frame (required)    -  a 4x4 homogeneous matrix 
            this.kin.setBaseFrame(frame);
        end
       
        function vertices = getHandleVerteces(this)
            vertices = [];
            for i= 1:size(this.handles,1)
                for j=1:2
                    vertices = [vertices; get(this.handles(i,j), 'Vertices')];
                end
            end
        end
    end
    
    methods(Access = private, Hidden = true)
        
        function [fk, fk_out] = updatePlot(this, angles, fbk)
        %UPDATEPLOT updates the link patches that were previously plotted
        %by initialPlot. 
        
            set(0, 'currentfigure', this.figure_h);       
        
            this.setBaseFrame(eye(4));
        
            fk = this.kin.getForwardKinematics('CoM',angles);
            fk_out = this.kin.getForwardKinematics('Output',angles);       
            
            [upper, lower, elbow, grip_mobile, grip_static] = this.loadMeshes();

            
            this.computeAndSetBaseFrame(fk, fk_out, fbk);

            fk = this.kin.getForwardKinematics('CoM', angles);
            fk_out = this.kin.getForwardKinematics('Output',angles);       

            h = this.handles;
            if(~ishandle(h(1,1)))
                error('Plotting window has been closed. Exiting program.');
            end
            angleInd = 1;
            for i=1:this.kin.getNumBodies()
                %For each body, check what type and handle based on that
                %type
                if(strcmp(this.jointTypes{i}{1}, 'FieldableElbowJoint'))
                    fv = this.transformSTL(lower, fk(:,:,i));
                    set(h(i,1), 'Vertices', fv.vertices(:,:));
                    set(h(i,1), 'Faces', fv.faces);

                    fv = this.transformSTL(upper, fk(:,:,i)*this.roty(angles(angleInd)));
                    set(h(i,2), 'Vertices', fv.vertices(:,:));
                    set(h(i,2), 'Faces', fv.faces);
                    angleInd = angleInd + 1;
                elseif(strcmp(this.jointTypes{i}{1}, ...
                        'FieldableStraightLink'))
                    fv = this.transformSTL(this.getCylinder(this.jointTypes{i}),...
                                           fk(:,:,i));
                    set(h(i,1), 'Vertices', fv.vertices(:,:));
                    set(h(i,1), 'Faces', fv.faces);
                elseif(strcmp(this.jointTypes{i}{1},'FieldableElbowLink'))
                    fv = this.transformSTL(elbow, fk_out(:,:,i));
                    set(h(i,1), 'Vertices', fv.vertices(:,:));
                    set(h(i,1), 'Faces', fv.faces);
                elseif(strcmp(this.jointTypes{i}{1}, 'Foot'))
                    fv = this.transformSTL(this.getSphere(this.jointTypes{i}),...
                                           fk_out(:,:,i-1));
                    set(h(i,1), 'Vertices', fv.vertices(:,:));
                    set(h(i,1), 'Faces', fv.faces);
                elseif(strcmp(this.jointTypes{i}{1}, 'FieldableElbowJoint'))
                    fv = this.transformSTL(lower, fk(:,:,i));
                    set(h(i,1), 'Vertices', fv.vertices(:,:));
                    set(h(i,1), 'Faces', fv.faces);
                elseif(strcmp(this.jointTypes{i}{1}, 'FieldableGripper'))
                    fv = this.transformSTL(lower, fk(:,:,i));
                    set(h(i,1), 'Vertices', fv.vertices(:,:));
                    set(h(i,1), 'Faces', fv.faces);

                    fv = this.transformSTL(grip_mobile, fk(:,:,i)*this.roty(angles(angleInd)));
                    set(h(i,2), 'Vertices', fv.vertices(:,:));
                    set(h(i,2), 'Faces', fv.faces);
                    
                    fv = this.transformSTL(grip_static, fk(:,:,i));
                    set(h(i,3), 'Vertices', fv.vertices(:,:));
                    set(h(i,3), 'Faces', fv.faces);
                    angleInd = angleInd + 1;
                end
                
                if i ==this.kin.getNumBodies()
                    cube = this.getCube();
                    GoProFace = this.getFace();
                    fv = this.transformSTL(cube, fk_out(:,:,i));
                    set(h(i,3), 'Vertices', fv.vertices);                                        
                    set(h(i,3), 'Faces', fv.faces);
                    fv = this.transformSTL(GoProFace, fk_out(:,:,i));
                    set(h(i,4), 'Vertices', fv.vertices);                                        
                    set(h(i,4), 'Faces', fv.faces);        
                    
                    if ~isempty(fbk)
                        norm_acc = norm([fbk.accelX(i), fbk.accelY(i), fbk.accelZ(i)]);                        
                        arrow = this.getArrow(norm_acc/9.81);
                        acc_dir = [fbk.accelX(i), fbk.accelY(i), fbk.accelZ(i)]/norm_acc;
                        r = vrrotvec( [1 0 0], acc_dir);                         m = vrrotvec2mat(r);
                        transf = eye(4); transf(1:3,1:3) = m;
                        fv = this.transformSTL(arrow, fk(:,:,i)*transf);
                        set(h(i,5), 'Vertices', fv.vertices);                                        
                        set(h(i,5), 'Faces', fv.faces);  
                        if  (isempty(this.accTrustability) || this.accTrustability(this.kin.getNumBodies-i+1)>0)
                            color_blue = 0;
                        else
                            color_blue = 1;
                        end
                        color = [min(max(abs(norm_acc/9.81 - 1),0),1) , 0, color_blue ];
                        set(h(i,3), 'FaceColor', color); 
                        set(h(i,3), 'EdgeColor', color);
                    end;
                else
                    if ~isempty(fbk)
                        norm_acc = norm([fbk.accelX(i), fbk.accelY(i), fbk.accelZ(i)]);                        
                        arrow = this.getArrow(norm_acc/9.81);
                        acc_dir = [fbk.accelX(i), fbk.accelY(i), fbk.accelZ(i)]/norm_acc;
                        r = vrrotvec( [1 0 0], acc_dir);                         m = vrrotvec2mat(r);
                        transf = eye(4); transf(1:3,1:3) = m;
                        fv = this.transformSTL(arrow, fk(:,:,i)*transf);
                        set(h(i,3), 'Vertices', fv.vertices);                                        
                        set(h(i,3), 'Faces', fv.faces);  
                        
                        if  (isempty(this.accTrustability) || this.accTrustability(this.kin.getNumBodies-i+1)>0)
                            color_blue = 0;
                        else
                            color_blue = 1;
                        end                       
                        color = [min(max(abs(norm_acc/9.81 - 1),0),1) , 0, color_blue];
                        set(h(i,3), 'FaceColor', color); 
                        set(h(i,3), 'EdgeColor', color);
                    end;
                end
            end
%             axis([-inf inf -inf inf -inf inf]);
        end
        
        function initializeKinematics(this, numLinks)
        %INITIALIZEKINEMATICS creates a default kinematics object
        %if one has not already been assigned.
            if(this.kin.getNumBodies > 0)
                return;
            end
            
            for i=1:numLinks
                this.kin.addBody('FieldableElbowJoint');
                this.jointTypes{i} = {'FieldableElbowJoint'};
            end
        end
        
        function setKinematicsFromJointTypes(this, types)
        %Creates a HebiKinematics object that will be used to calculate the
        %Forward Kinematics when plotting. 
        %types  is a struct of structs. Each struct can be fed into
        %HebiKinematics as a link type with necessary parameters
        %There is a custom type of "Foot" for plotting the end caps.
            this.kin = HebiKinematics();
            this.jointTypes = types;
            if(length(types) == 0)
                return;
            end
            for i = 1:length(types)
                if(strcmp(types{i}{1}, 'Foot'))
                    this.kin.addBody('FieldableStraightLink', ...
                        types{i}{2:end});
                else
                    this.kin.addBody(types{i}{:});
                end
            end
        end

        function [fk, fk_out] = initialPlot(this, angles, fbk)
        %INITIALPLOT creates patches representing the CAD of the
        %manipulator, sets lighting, and labels graph

            this.figure_h = figure(42);
        
            this.initializeKinematics(length(angles));
            n = this.kin.getNumBodies();
            
            this.handles = zeros(n, 5);

            [upper, lower, elbow, grip_mobile, grip_static]...
                            = this.loadMeshes();
            
            fk = this.kin.getForwardKinematics('CoM', angles);
            fk_out = this.kin.getForwardKinematics('Output', angles);

            
            this.frame = eye(4);
            this.computeAndSetBaseFrame(fk, fk_out, fbk);
            
            fk = this.kin.getForwardKinematics('CoM', angles);
            fk_out = this.kin.getForwardKinematics('Output', angles);

            
            if(strcmp(this.lighting, 'on'))
                light('Position',[0,0,10]);
                light('Position',[5,0,10]);
                light('Position',[-5,0,10]);
                lightStyle = 'gouraud';
                strength = .3;
            elseif(strcmp(this.lighting, 'far'))
                c = [.7,.7,.7];
                light('Position',[0,0,100],'Color',c);
                light('Position',[-100,0,0],'Color',c);
                light('Position',[100,0,0],'Color',c);
                light('Position',[0,-100,0], 'Color',c);
                light('Position',[0,100,0],'Color',c);
                lightStyle = 'flat';
                strength = 1.0;
            else
                lightStyle = 'flat';
                strength = 1.0;
            end
            
            angleInd = 1;
            for i=1:this.kin.getNumBodies
                if(strcmp(this.jointTypes{i}{1}, 'FieldableElbowJoint'))
                    this.handles(i,1:2) = ...
                        this.patchHebiElbow(lower, upper, fk(:,:,i), ...
                                       angles(angleInd), lightStyle, ...
                                            strength);
                    angleInd = angleInd + 1;
                elseif(strcmp(this.jointTypes{i}{1}, 'FieldableElbowLink'))
                    this.handles(i,1) = ...
                        this.patchElbowLink(elbow, fk_out(:,:,i));
                elseif(strcmp(this.jointTypes{i}{1}, ...
                        'FieldableStraightLink'))
                    this.handles(i,1) = ...
                        this.patchCylinder(fk(:,:,i), this.jointTypes{i});
                elseif(strcmp(this.jointTypes{i}{1}, 'Foot'))
                    this.handles(i,1) = ...
                        this.patchSphere(fk(:,:,i), this.jointTypes{i});
                elseif(strcmp(this.jointTypes{i}{1}, 'FieldableGripper'))
                    this.handles(i,:) = ...
                        this.patchHebiGripper(lower,grip_mobile,grip_static, fk(:,:,i), ...
                                       angles(angleInd), lightStyle, ...
                                            strength);
                    angleInd = angleInd + 1;
                end
                if i==this.kin.getNumBodies
                    
                    cube = this.getCube();
                    this.handles(i,3) = ...
                        patch(this.transformSTL(cube, fk_out(:,:,i)), ...
                              'FaceColor', [.2,.2,.2],...          
                              'EdgeColor', [0,0,0],...
                              'FaceLighting', lightStyle, ...
                              'AmbientStrength', strength);
                          
                    GoProFace = this.getFace();                          
                    this.handles(i,4) = ...
                        patch(this.transformSTL(GoProFace, fk_out(:,:,i)), ...
                              'FaceColor', [1,1,1],...          
                              'EdgeColor', [0,0,0], ...
                              'FaceLighting', lightStyle, ...
                              'AmbientStrength', strength);
                    
                    if ~isempty(fbk)
                        norm_acc = norm([fbk.accelX(i), fbk.accelY(i), fbk.accelZ(i)]);                        
                        arrow = this.getArrow(norm_acc/9.81);
                        acc_dir = [fbk.accelX(i), fbk.accelY(i), fbk.accelZ(i)]/norm_acc;
                        r = vrrotvec([1 0 0],acc_dir);                        m = vrrotvec2mat(r);
                        transf = eye(4); transf(1:3,1:3) = m;
                        fv = this.transformSTL(arrow, fk(:,:,i)*transf);  
                        
                        if ((isempty(this.accTrustability) || this.accTrustability(this.kin.getNumBodies-i+1)>0)  )
                            face_color = [0 0 0];
                            edge_color = [0 0 0];
                        else
                            face_color = [0 0 1];
                            edge_color = [0 0 1];
                        end
                        
                        this.handles(i,5) = ...
                            patch(fv, ...
                                  'FaceColor', face_color,...          
                                  'EdgeColor', edge_color, ...
                                  'FaceLighting', lightStyle, ...
                                  'AmbientStrength', strength);   
                    end
                else
%                     arrow = this.getArrow();
                    if ~isempty(fbk)
                        norm_acc = norm([fbk.accelX(i), fbk.accelY(i), fbk.accelZ(i)]);                        
                        arrow = this.getArrow(norm_acc/9.81);
                        acc_dir = [fbk.accelX(i), fbk.accelY(i), fbk.accelZ(i)]/norm_acc;
                        r = vrrotvec([1 0 0],acc_dir);                        m = vrrotvec2mat(r);
                        transf = eye(4); transf(1:3,1:3) = m;
                        fv = this.transformSTL(arrow, fk(:,:,i)*transf);                
                        if ((isempty(this.accTrustability) || this.accTrustability(this.kin.getNumBodies-i+1)>0)  )
                            face_color = [0 0 0];
                            edge_color = [0 0 0];
                        else
                            face_color = [0 0 1];
                            edge_color = [0 0 1];
                        end
                        
                        this.handles(i,3) = ...
                            patch(fv, ...
                                  'FaceColor', face_color,...          
                                  'EdgeColor', edge_color, ...
                                  'FaceLighting', lightStyle, ...
                                  'AmbientStrength', strength);   
                    end
                end
            end
            axis('image');
            view([45, 35]);
            xlabel('x');
            ylabel('y');
            zlabel('z');
        end
        
        function h = patchHebiElbow(this,lower, upper, fk, angle, ...
                                    lightStyle, strength)
        %Patches (plots) the HebiKinematics elbow joint
            h(1,1) =  ...
                patch(this.transformSTL(lower, fk), ...
                      'FaceColor', [.5,.1,.2],...
                      'EdgeColor', 'none',...
                      'FaceLighting', lightStyle, ...
                      'AmbientStrength', strength);
            h(1,2) = ...
                patch(this.transformSTL(upper, fk*this.roty(angle)), ...
                'FaceColor', [.5,.1,.2],...
                'EdgeColor', 'none',...
                'FaceLighting', lightStyle, ...
                'AmbientStrength', strength);
        end
        
        function h = patchHebiGripper(this,lower,grip_mobile,...
                                    grip_static,fk, angle, ...
                                    lightStyle, strength)
        %Patches (plots) the SEA HebiGripper
            h(1,1) =  ...
                patch(this.transformSTL(lower, fk), ...
                      'FaceColor', [.5,.1,.2],...
                      'EdgeColor', 'none',...
                      'FaceLighting', lightStyle, ...
                      'AmbientStrength', strength);
                  
            h(1,2) = ...
                patch(this.transformSTL(grip_mobile, fk*this.roty(angle)), ...
                'FaceColor', [.5, .5, .5],...
                'EdgeColor', 'none',...
                'FaceLighting', lightStyle, ...
                'AmbientStrength', .1);      
                  
            h(1,3) = ...
                patch(this.transformSTL(grip_static, fk), ...
                'FaceColor', [.5, .5, .5],...
                'EdgeColor', 'none',...
                'FaceLighting', lightStyle, ...
                'AmbientStrength', .1);
        end
        
        function h = patchElbowLink(this,elbow, fk)
        %Patches (plots) the HebiKinematics elbow joint
            h(1,1) =  ...
                patch(this.transformSTL(elbow, fk), ...
                      'FaceColor', [.5,.5,.5],...
                      'EdgeColor', 'none',...
                      'FaceLighting', 'gouraud', ...
                      'AmbientStrength', .1);
                  
        end
        
        function [upper, lower, elbow, grip_mobile, grip_static]...
                 = loadMeshes(this)
        %Returns the relevent meshes
        %Based on low_res different resolution meshes will be loaded
            stldir = [fileparts(mfilename('fullpath')), '/stl'];
            
            if(this.lowResolution)
                meshes = load([stldir, '/FieldableKinematicsPatchLowRes.mat']);
            else
                meshes = load([stldir, '/FieldableKinematicsPatch.mat']);
            end
            lower = meshes.lower;
            upper = meshes.upper;
            elbow = meshes.elbow;
            grip_mobile = meshes.grip_mobile;
            grip_static = meshes.grip_static;
        end
        
        function h = patchCylinder(this, fk, types)
        %patches (plots) a cylinder and returns the handle
            h = patch(this.transformSTL(this.getCylinder(types), fk), ...
                      'FaceColor', [.5, .5, .5],...
                      'EdgeColor', 'none',...
                      'FaceLighting', 'gouraud',...
                      'AmbientStrength',.1);
        end
        
        function cyl = getCylinder(this, types)
        %Returns the patch faces and vertices of a cylinder
            p = inputParser();
            p.addParameter('ext1', .4);
            p.addParameter('twist', 0);
            parse(p, types{2:end});
            r = .025;
            h = p.Results.ext1 + .015; %Add a bit for connection section
            [x,y,z] = cylinder;
            cyl = surf2patch(r*x, r*y, h*(z -.5));
        end
        
        function h = patchSphere(this, fk, types)
        %patches (plots) a sphere and returns the handle
            h = patch(this.transformSTL(this.getSphere(types), fk),...
                      'FaceColor', [0, 0, 0],...
                      'EdgeColor', 'none',...
                      'FaceLighting', 'gouraud',...
                      'AmbientStrength',.1);
        end
        
        function sph = getSphere(this, types)
        %Returns the patch faces and vertices of a sphere
            p = inputParser();
            p.addParameter('ext1', .4);
            p.addParameter('twist', 0);
            parse(p, types{2:end});
            r = p.Results.ext1;
            [x,y,z] = sphere;
            sph = surf2patch(r*x, r*y, r*z );
        end
        
        function fv = transformSTL(this, fv, trans)
        %Transforms from the base frame of the mesh to the correctly
        %location in space
            fv.vertices = (trans * [fv.vertices, ones(size(fv.vertices,1), ...
                                                      1)]')';
            fv.vertices = fv.vertices(:,1:3);

        end
        
        function computeAndSetBaseFrame(this, fk, fk_out, fbk)
            if(strcmpi(this.frameType, 'VC'))
                this.frame = this.frame*unifiedVC(fk, eye(3), eye(3));
                this.setBaseFrame(inv(this.frame));
            elseif (strcmpi(this.frameType, 'gravity'))
                this.CF.update(fbk);
                tailInGravity = this.CF.getInGravity('tail');
                if isempty(tailInGravity)
                    tailInGravity = eye(4);
                end
                %The module reference frames in CF are aligned for zero
                %joint angle, while in HebiKinematics they rotate from
                %tail to head of pi/2 per module around the z-axis 
                %(screw convention)
                %There is also an offset of -pi to rotate from one
                %convention to the other                   
                tailInGravity_screwConvention = tailInGravity*...
                                            this.rotz(-pi+length(fk)*pi/2);
                this.setBaseFrame(tailInGravity_screwConvention);
%                 this.frame =
            elseif (strcmpi(this.frameType, 'head'))
%                 this.Thead;
%                 inv(fk_out(:,:,end));
                this.setBaseFrame(this.Thead*inv(fk_out(:,:,end)));
            end
        end
        

        function m = roty(this, theta)
        %Homogeneous transform matrix for a rotation about y
            m = [cos(theta),  0, sin(theta), 0;
                 0,           1, 0,          0;
                 -sin(theta), 0, cos(theta), 0;
                 0,           0, 0,          1];
        end
        
        function m = rotz(this, theta)
        %Homogeneous transform matrix for a rotation about z
            m = [cos(theta), -sin(theta), 0, 0;
                 sin(theta),  cos(theta), 0, 0;
                 0          , 0, 1,          0;
                 0,           0, 0,          1];
        end
        
        function cube = getCube(this)
            
            cube.vertices = [.5 .5 0;  -.5 .5 0; -.5 -.5 0; .5 -.5 0; ...
                .5 .5 1;  -.5 .5 1; -.5 -.5 1; .5 -.5 1;
                0 -.3 1; -.3 -.2 1; -.3 .2 1; 0 .3  1 ];
            cube.faces = [1 2 3 4; 5 6 7 8; 1 5 8 4; 8 7 3 4; 7 6 2 3; 6 5 1 2];
            cube.vertices = cube.vertices*0.03;
        end
        
        function GoProFace = getFace(this)
            GoProFace.vertices = [ .1 -.3 1.01; .3 -.2 1.01; .3 .2 1; .1 .3  1.01 ; ...
                                 -.1 -.2 1.01; -.3 -.2 1.01; -.3 -.1 1.01; -.1 -.1 1.01; ...
                                 -.1 .2 1.01; -.3 .2 1.01; -.3 .1 1.01; -.1 .1 1.01 ];
            GoProFace.faces = [1 2 3 4; 5 6 7 8; 9 10 11 12];
            GoProFace.vertices = GoProFace.vertices*0.03;            
        end
        
        function arrow = getArrow(this, scale)
            %% Setup the intial values
            noPoints = 20;
            theta = linspace(0,2*pi,noPoints);
            radius = 0.002;
            height = 0.1;
            
            if ~isempty(scale)
                height = height*scale;
            end

            %% Create the X and Y vectorrs
            x = radius.*cos(theta);
            y = radius.*sin(theta);

            %% create matrix of bottom and top points 
            bottom = [repmat(-height,noPoints,1), x',y'];
            top = [repmat(height,noPoints,1), x',y']; 

            %% Create faces and vertices matrix
            %create the vertices matrix
            arrow.vertices = [bottom;top];
            %Create the faces matrix;
            %Add the 'Top' triangles
            faces = [(1:noPoints)',(noPoints+1:2*noPoints)',((noPoints+1:2*noPoints)+1)'];
            %replace the last to one since it is out of bounds
            faces(end,3) = 1;
            %Add the 'Bottom Triangles'

            faces = [faces;...
                (1:noPoints)',(2:noPoints+1)',((noPoints+1:2*noPoints)+1)'];
            %replace the last to one since it is wrong
            faces(end,2) = 1;
            faces(end,3) = noPoints+1;

            
            arrow.faces = faces;

        end
        
    end
    
    properties(Access = private, Hidden = true)
        kin;
        jointTypes;
        handles;
        lowResolution;
        firstRun;
        lighting;
        frameType;
        frame;
        drawNow;
        CF;
        readyToPlot;
        firstPlot;
        Thead;
        accelOffsets;
        gyroOffsets;
        gyrosTrustability;
        accTrustability;
        GoPro;
        figure_h;
    end
end
