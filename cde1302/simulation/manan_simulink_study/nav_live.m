function nav_live(cmd,varargin)
%NAV_LIVE Optional live-view hook called by nav_estimator during simulation.
%   nav_live('set',fcn,periodS)  call fcn(t,pos,quat,diag) every periodS of
%                                simulated time (the GUI uses this)
%   nav_live('clear')            stop calling it
%   nav_live('tick',t,pos,quat,diag)  called by the estimator; no-op unless set
% Display only: the callback receives copies and cannot change the simulation.
persistent fcn period lastT
switch cmd
    case 'set'
        fcn=varargin{1}; period=varargin{2}; lastT=-inf;
    case 'clear'
        fcn=[];
    case 'tick'
        if isempty(fcn), return; end
        t=varargin{1};
        if t-lastT<period && t>0, return; end
        lastT=t;
        try
            fcn(varargin{:});
        catch err
            fcn=[];   % a broken display must never stop the simulation
            warning('MananStudy:LiveView','Live view disabled: %s',err.message);
        end
end
end
