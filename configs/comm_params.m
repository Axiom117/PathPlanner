%% comm_params.m - Communication Configuration Function
function config = comm_params()
    % Communication Parameters Configuration
    % Returns a structure containing all communication-related parameters
    
    % TCP/IP Settings
    config.controllerHost = '10.8.251.124';   % Controller host IP
    config.controllerPort = 5000;          % Controller port number
    config.connectionTimeout = 10;         % Connection timeout [s]
    config.responseTimeout = 1;            % Response timeout [s]
    
    % Protocol Settings
    config.protocolVersion = 'v1.1';       % Protocol version
    config.messageBufferSize = 1024;       % Message buffer size [bytes]
    config.heartbeatInterval = 30;         % Heartbeat interval [s]
    config.commandDelimiter = '\r\n';      % Command delimiter
    
    % Error Handling
    config.maxRetryAttempts = 3;           % Maximum retry attempts
    config.retryDelay = 1;                 % Delay between retries [s]
    config.enableAutoReconnect = true;     % Enable automatic reconnection
    config.reconnectInterval = 5;          % Reconnection attempt interval [s]
    
    % Security Settings
    config.enableEncryption = false;       % Enable message encryption
    config.authenticationRequired = false; % Require authentication
    config.maxConnections = 5;             % Maximum concurrent connections
    
    % Logging Settings
    config.enableCommLogging = true;       % Enable communication logging
    config.logCommands = true;             % Log sent commands
    config.logResponses = true;            % Log received responses
    config.commLogLevel = 'INFO';          % Logging level (DEBUG, INFO, WARN, ERROR)
    
    % Performance Settings
    config.enableNagle = false;            % Enable Nagle's algorithm
    config.socketBufferSize = 8192;        % Socket buffer size [bytes]
    config.keepAliveEnabled = true;        % Enable TCP keep-alive
    config.keepAliveInterval = 60;         % Keep-alive interval [s]
    
    fprintf('Communication configuration loaded with %d parameters\n', length(fieldnames(config)));
end