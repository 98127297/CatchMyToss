classdef UR3 < handle
    properties
        %> Robot model
        model;
        
        %> workspace
        workspace = [-1 1 -1 1 0 1];   
               
        %> If we have a tool model which will replace the final links model, combined ply file of the tool model and the final link models
        toolModelFilename = []; % Available are: 'DabPrintNozzleTool.ply';        
        toolParametersFilename = []; % Available are: 'DabPrintNozzleToolParameters.mat';        
    end
    
    methods%% Class for UR3 robot simulation
        function self = UR3(toolModelAndTCPFilenames)
            if 0 < nargin
                if length(toolModelAndTCPFilenames) ~= 2
                    error('Please pass a cell with two strings, toolModelFilename and toolCenterPointFilename');
                end
                self.toolModelFilename = toolModelAndTCPFilenames{1};
                self.toolParametersFilename = toolModelAndTCPFilenames{2};
            end
            
            self.GetUR3Robot();
%             self.PlotAndColourRobot();%robot,workspace);

            drawnow            
            % camzoom(2)
            % campos([6.9744    3.5061    1.8165]);

%             camzoom(4)
%             view([122,14]);
%             camzoom(8)
%             teach(self.model);
        end

        %% GetUR3Robot
        % Given a name (optional), create and return a UR3 robot model
        function GetUR3Robot(self)
            pause(0.001);
            name = ['UR_3_',datestr(now,'yyyymmddTHHMMSSFFF')];

            L1 = Link('d',0.1519,'a',0,'alpha',pi/2,'offset',0,'qlim', [-2*pi,2*pi])
            L2 = Link('d',0,'a',-0.24365,'alpha',0,'offset',0,'qlim', [-2*pi,2*pi])
            L3 = Link('d',0,'a',-0.21325,'alpha', 0,'offset',0,'qlim', [-2*pi,2*pi])
            L4 = Link('d',0.11235,'a',0,'alpha',pi/2,'offset',0,'qlim', [-2*pi,2*pi])
            L5 = Link('d',0.08535,'a',0,'alpha',-pi/2,'offset',0,'qlim', [-2*pi,2*pi])
            L6 = Link('d',0.0819,'a',0,'alpha',0,'offset',0,'qlim', [-2*pi,2*pi])

            self.model = SerialLink([L1 L2 L3 L4 L5 L6],'name',name);
        end

        
        %% PlotAndColourRobot
        % Given a robot index, add the glyphs (vertices and faces) and
        % colour them in if data is available 
        function PlotAndColourRobot(self)%robot,workspace)
            for linkIndex = 0:self.model.n
                [ faceData, vertexData, plyData{linkIndex + 1} ] = plyread(['UR3Link',num2str(linkIndex),'.ply'],'tri'); %#ok<AGROW>
                self.model.faces{linkIndex + 1} = faceData;
                self.model.points{linkIndex + 1} = vertexData;
            end

            if ~isempty(self.toolModelFilename)
                [ faceData, vertexData, plyData{self.model.n + 1} ] = plyread(self.toolModelFilename,'tri'); 
                self.model.faces{self.model.n + 1} = faceData;
                self.model.points{self.model.n + 1} = vertexData;
%                 toolParameters = load(self.toolParametersFilename);
%                 self.model.tool = toolParameters.tool;
%                 self.model.qlim = toolParameters.qlim;
%                 warning('Please check the joint limits. They may be unsafe')
            end
            % Display robot
            self.model.plot3d(zeros(1,self.model.n),'noarrow','workspace',self.workspace);
            if isempty(findobj(get(gca,'Children'),'Type','Light'))
                camlight
            end  
            self.model.delay = 0;

            % Try to correctly colour the arm (if colours are in ply file data)
            for linkIndex = 0:self.model.n
                handles = findobj('Tag', self.model.name);
                h = get(handles,'UserData');
                try 
                    h.link(linkIndex+1).Children.FaceVertexCData = [plyData{linkIndex+1}.vertex.red ...
                                                                  , plyData{linkIndex+1}.vertex.green ...
                                                                  , plyData{linkIndex+1}.vertex.blue]/255;
                    h.link(linkIndex+1).Children.FaceColor = 'interp';
                catch ME_1
                    disp(ME_1);
                    continue;
                end
            end
        end        
    end
end