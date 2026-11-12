function A = anchor_layout(name,cfg)
%ANCHOR_LAYOUT Preset known-landmark layouts, N x 3 [id x y] in arena metres.
%   Floor tags only: the camera looks straight down, so a wall tag is never
%   in view. Ids follow the team (22-28 field tags, 29 = home tag).
%   No preset is claimed to be optimal; they are starting points to compare.
%
%   'none'                 no landmarks (Method A only)
%   'home'                 home tag under the take-off spot
%   'corners4'             four search-area corners, 0.15 m inside
%   'corners_centre5'      corners + centre (the brief's "do not assume optimal" case)
%   'home_corners5'        home + corners
%   'home_corners_centre6' home + corners + centre
%   'lane_ends'            home + one tag at the far end of every other lane
names={'none','home','corners4','corners_centre5','home_corners5', ...
    'home_corners_centre6','lane_ends'};
if nargin==0, A=names; return; end
b=cfg.geometry.searchBounds; inset=0.15;
x=[b(1,1)+inset b(1,2)-inset]; y=[b(2,1)+inset b(2,2)-inset];
corners=[x(1) y(1);x(2) y(1);x(2) y(2);x(1) y(2)];
centre=mean(b,2).';
home=cfg.frames.homeArena(:).';
switch name
    case 'none', xy=zeros(0,2); ids=zeros(0,1);
    case 'home', xy=home; ids=29;
    case 'corners4', xy=corners; ids=(22:25).';
    case 'corners_centre5', xy=[corners;centre]; ids=(22:26).';
    case 'home_corners5', xy=[home;corners]; ids=[29;(22:25).'];
    case 'home_corners_centre6', xy=[home;corners;centre]; ids=[29;(22:26).'];
    case 'lane_ends'
        r=cfg.mission.routeArena; ends=r(2:4:end,:); % far end of lanes 1,3,5...
        ends(:,1)=min(max(ends(:,1),x(1)),x(2));
        xy=[home;ends]; ids=[29;(22:21+size(ends,1)).'];
    otherwise
        error('MananStudy:AnchorLayout','Unknown anchor layout %s. Options: %s', ...
            name,strjoin(names,', '));
end
A=[ids(:),xy];
end
