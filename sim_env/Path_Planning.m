function [Target, Stop] = Path_Planning(State, Waypoints)


persistent WP_List;
persistent N_WPs;
persistent Target_WP_Idx;


Stop = 0;


%% Configs:

% Distance from WP that Counts as Visited:
WP_Thresh = 5;

% How Far Forward the RVWP should be from the Robot's Projected Position:
LookAhead = 10;


% X-Y Position of the Robot:
Pos = [State(1), State(2)];


%% Parsing and Initialisation of Waypoints:

if isempty(WP_List)

    N_WPs = size(Waypoints, 1);

    WP_List = repmat(struct("Pos", zeros(1, 2), "Visited", false, "Vect", zeros(1, 2), "Len", 0, "Dir", zeros(1, 2)), 1, N_WPs);

    % Turning the List of WPs into a Struct Array:
    for k = 1:N_WPs

        WP_List(k).Pos = Waypoints(k, :);

        if k ~= 1
            WP_List(k).Vect = Waypoints(k, :) - Waypoints(k-1, :);
            WP_List(k).Len = norm(WP_List(k).Vect);
            WP_List(k).Dir = WP_List(k).Vect / WP_List(k).Len;
        else
            WP_List(k).Vect = Waypoints(k, :);
            WP_List(k).Len = norm(WP_List(k).Vect);
            WP_List(k).Dir = WP_List(k).Vect / WP_List(k).Len;
        end
    
    end

    Target_WP_Idx = 1;
end




%% Indexing Target WP:

if sqrt( (WP_List(Target_WP_Idx).Pos(1)-State(1))^2 + (WP_List(Target_WP_Idx).Pos(2)-State(2))^2 ) < WP_Thresh

    WP_List(Target_WP_Idx).Visited = true;

    if (Target_WP_Idx == N_WPs), Stop = 1;
    else, Target_WP_Idx = Target_WP_Idx + 1;
    end

end



%% Calculating Vector Projection onto Segment + LookAhead:


Dot_Product_Proj =  dot(WP_List(Target_WP_Idx).Vect, (Pos - WP_List(Target_WP_Idx).Pos)) / WP_List(Target_WP_Idx).Len;

Target = WP_List(Target_WP_Idx).Pos + ((Dot_Product_Proj + LookAhead) * WP_List(Target_WP_Idx).Dir);