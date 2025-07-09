classdef PathPlannerEventData < event.EventData
    % PathPlannerEventData - Event data container for PathPlanner system
    % 
    % This class encapsulates event information passed between modules
    % and to external event listeners. It provides consistent structure
    % for all PathPlanner events.
    %
    % Properties:
    %   Message   - Descriptive text about the event
    %   Success   - Boolean indicating operation success/failure
    %   Data      - Additional data payload (optional)
    %   Timestamp - When the event occurred
    %
    % Usage:
    %   eventData = PathPlannerEventData('Connection established');
    %   eventData = PathPlannerEventData('Error occurred', false);
    %   eventData = PathPlannerEventData('Trajectory ready', true, trajectoryStruct);
    
    properties (SetAccess = private)
        Message         % Event description message
        Success         % Operation success flag
        Data            % Additional event data payload
        Timestamp       % Event occurrence timestamp
    end
    
    methods
        function obj = PathPlannerEventData(message, success, data)
            % Constructor - Create event data with message and optional parameters
            %
            % Inputs:
            %   message - (string) Descriptive event message
            %   success - (logical, optional) Success flag, default true
            %   data    - (any, optional) Additional data payload, default empty
            
            if nargin < 2
                success = true;
            end
            if nargin < 3
                data = [];
            end
            
            % Store event information
            obj.Message = message;
            obj.Success = success;
            obj.Data = data;
            obj.Timestamp = datetime('now');
        end

        function str = toString(obj)
            % Convert event to string representation for logging
            % Returns: str - Formatted string with timestamp and message
            
            timeStr = datestr(obj.Timestamp, 'HH:MM:SS.FFF');
            statusStr = '';
            
            if ~obj.Success
                statusStr = ' [ERROR]';
            end
            
            str = sprintf('[%s]%s %s', timeStr, statusStr, obj.Message);
        end
        
        function disp(obj)
            % Display event information in command window
            
            fprintf('%s\n', obj.toString());
            
            if ~isempty(obj.Data)
                fprintf('  Data: %s\n', mat2str(obj.Data));
            end
        end
    end
end