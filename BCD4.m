clear all
close all 
x_out = [-8 4 11 11 10 10 20 10 -5 -10 -7 -7]; %clockwise
y_out = [10 8 10 8 3 1 0 -12 -11 -12 0 1];
%number of holes has to be same
x_hole(1, :) = [ -4 -5, 0 4 3 3 4 -1 -1 -5 -4]; %c.clockwise
y_hole(1, :)= [ 1 0, -1 0 1 1.5 3 3 5 5 4];
x_hole(2, :) = [10, 10, 12, 12 NaN NaN NaN NaN NaN NaN NaN]; %c.clockwise
y_hole(2, :) = [-5, -6, -6, -5 NaN NaN NaN NaN NaN NaN NaN];
x_hole(3, :) = [-1, -1, 1, 1 NaN NaN NaN NaN NaN NaN NaN]; %c.clockwise
y_hole(3, :) = [-5, -6, -6, -5 NaN NaN NaN NaN NaN NaN NaN];
pgon = polyshape({x_out, x_hole(1, :), x_hole(2, :), x_hole(3, :)},{y_out, y_hole(1, :), y_hole(2,:), y_hole(3,:)});
plot(pgon)
[outer_event, outer_vertex, inner_event, inner_vertex] = createEvents(x_out, y_out, x_hole, y_hole);
%Start algorithm
x_current = -inf;
polygon_nr = 0;
times = 0;
edge_list = [];
created_polygons = [];

[event_nr, event_nr2, outer, first_vertex] = find_next_vertex(x_current, inner_event, outer_event, 0, 0, true);
%Assuming first vertex is always outside
edge_list(1, :)=[outer_event(event_nr).location outer_vertex(outer_event(event_nr).floor).location];
if ((outer_event(event_nr).location2(2))~=(outer_event(event_nr).location(2)))
    edge_list(2, :)=[outer_event(event_nr).location2 outer_vertex(outer_event(event_nr).ceiling).location];
else
    edge_list(2, :)= [outer_event(event_nr).location outer_vertex(outer_event(event_nr).ceiling).location];
end

outer_event(event_nr).closed = true; 
times = 0;
while (~isempty(edge_list)) %times < 6%
    [edge_list, edge_upper_number, edge_lower_number, edge_upper, edge_lower, inner_event, outer_event] = getEdges(edge_list, inner_event, outer_event, inner_vertex, outer_vertex);
    [polygon_nr, vertex_nr, created_polygons] = createNewPolygon(polygon_nr, created_polygons, edge_upper, edge_lower); 
    [event_nr, event_nr2, outer, first_vertex] = find_next_vertex(x_current, inner_event, outer_event, created_polygons(polygon_nr, 1), created_polygons(polygon_nr, vertex_nr-1), false);
    if (outer)
        x_current_next = outer_event(event_nr).location(1);
        type = outer_event(event_nr).type;
    else 
        x_current_next = inner_event(event_nr, event_nr2).location(1);
        type = inner_event(event_nr, event_nr2).type;
    end
    upper_number = 1;
    upper_vertices = [];
    x_current_new = x_current;
    while (type == 2) 
        x_current_new = x_current_next;
        if (upper_number == 1)
            [event_nr, event_nr2, outer, first_vertex] = find_next_vertex(x_current_next, inner_event, outer_event, created_polygons(polygon_nr, 1), created_polygons(polygon_nr, vertex_nr-1), false);
        else
            [event_nr, event_nr2, outer, first_vertex] = find_next_vertex(x_current_next, inner_event, outer_event, upper_vertices(upper_number-1), created_polygons(polygon_nr, vertex_nr-1), false);
        end
        if (~outer)
           [x_current_next, upper_vertices, upper_number, created_polygons, vertex_nr, edge_list, inner_event] = createInnerVertices(polygon_nr, first_vertex, inner_event, inner_vertex, event_nr, event_nr2, upper_number, upper_vertices, created_polygons, vertex_nr, edge_list);
           type = inner_event(event_nr, event_nr2).type;
        else
           x_current_next = outer_event(event_nr).location(1);
           if (outer_event(event_nr).type == 2)
            [upper_vertices, upper_number, created_polygons, vertex_nr, edge_list, outer_event] = createOuterVertices(polygon_nr, outer_event, event_nr, upper_number, upper_vertices, created_polygons, vertex_nr, edge_list, outer_vertex);
           end
           type = outer_event(event_nr).type;
        end
    end
    if (~outer)
        x_current_next = inner_event(event_nr, event_nr2).location(1);
        if (inner_event(event_nr, event_nr2).type == 1)
            [created_polygons, edge_list, inner_event]=innerPolygonEnd(x_current_next,first_vertex, outer_event, inner_event, event_nr, event_nr2, created_polygons, polygon_nr, upper_vertices, upper_number, edge_list, edge_upper_number, edge_lower_number, vertex_nr);
        elseif (inner_event(event_nr, event_nr2).type == 0)
            [edge_list, created_polygons]= newObstacle(created_polygons, polygon_nr, x_current_next, edge_list, edge_lower_number, edge_upper_number, vertex_nr, upper_vertices, inner_event(event_nr, event_nr2), inner_vertex(event_nr, inner_event(event_nr, event_nr2).ceiling).location, inner_vertex(event_nr, inner_event(event_nr, event_nr2).floor).location);
            inner_event(event_nr, event_nr2).closed = true;
        end
    else
        %Opening
        if(outer_event(event_nr).type == 0)
            x_current_next = outer_event(event_nr).location(1);
            [edge_list, created_polygons]= newObstacle(created_polygons, polygon_nr, x_current_next, edge_list, edge_lower_number, edge_upper_number, vertex_nr, upper_vertices, outer_event(event_nr), outer_vertex(outer_event(event_nr).ceiling).location, outer_vertex(outer_event(event_nr).floor).location);
            outer_event(event_nr).closed = true;
            if (upper_number == 1)
                [event_nr, event_nr2, outer, first_vertex] = find_next_vertex(x_current_next, inner_event, outer_event, created_polygons(polygon_nr, 1), created_polygons(polygon_nr, vertex_nr-1), false);
            else
                [event_nr, event_nr2, outer, first_vertex] = find_next_vertex(x_current_next, inner_event, outer_event, upper_vertices(upper_number-1), created_polygons(polygon_nr, vertex_nr-1), false);
            end
        %Closing
        elseif(outer_event(event_nr).type == 1)
            x_current_next = outer_event(event_nr).location(1);
            [created_polygons, edge_list] = closePolygon(x_current_next, edge_list, created_polygons, polygon_nr, vertex_nr, upper_vertices,edge_lower_number, edge_upper_number);
            [edge_list] = updateEdgeList(edge_list, outer_event(event_nr), [], [], true);
            if (outer_event(event_nr).upper)
                outer_event(event_nr).closed = true;
            else
                outer_event(event_nr).upper = true;
            end
        end
    end
    times = times+1;
end
drawPolygons(created_polygons);
function [nr1, nr2, outer, first_vertex] = find_next_vertex(x_current, inner_event, outer_event, vertex1, vertex2, no_border)
    x_search = inf;
    nr1 = 1;
    nr2 = 1;
    outer = true;
    type = -1;
    y_current = inf;
    if (no_border)
        y1 = inf;
        y2 = -inf;
    end 
    for i=1:size(outer_event, 2)
        if (outer_event(i).location(1)<=x_search && outer_event(i).location(1)>=x_current && ~outer_event(i).closed)
            if (outer_event(i).type > type || outer_event(i).location(1)<x_search||outer_event(i).location(2)>y_current)
                if (~no_border) 
                    y1 = calculateVertex(outer_event(i).location(1), vertex1.point, vertex1.direction);
                    y2 = calculateVertex(outer_event(i).location(1), vertex2.point, vertex2.direction);
                end
                if (outer_event(i).location(2)<=y1 && outer_event(i).location(2)>=y2)
                    x_search= outer_event(i).location(1);
                    nr1 = i;
                    outer = true;
                    first_vertex = true;
                    type = outer_event(i).type;
                    %y_current = outer_event(i).location(2);
                elseif (outer_event(i).location2(2)<=y1 && outer_event(i).location2(2)>=y2)
                    x_search= outer_event(i).location(1);
                    nr1 = i;
                    outer = true;
                    first_vertex = false;
                    type = outer_event(i).type;
                    %y_current = outer_event(i).location(2);
                end
            end
        end
    end
    for a = 1:size(inner_event, 1)
        for i=1:size(inner_event(a, :), 2)
            if (isempty(inner_event(a, i).location))
                break;
            end
            if (inner_event(a, i).location(1) <= x_search && inner_event(a,i).location(1)>=x_current && ~inner_event(a,i).closed)
                if ((inner_event(a, i).type > type) || inner_event(a, i).location(1) < x_search || inner_event(a,i).location(2)>y_current)
                    if (~no_border) 
                        y1 = calculateVertex(inner_event(a, i).location(1), vertex1.point, vertex1.direction);
                        y2 = calculateVertex(inner_event(a, i).location(1), vertex2.point, vertex2.direction);
                    end
                    if (inner_event(a,i).location(2)<=y1 && inner_event(a,i).location(2)>=y2)
                        x_search  = inner_event(a, i).location(1);
                        nr1 =a;
                        nr2 =i;
                        outer = false;
                        first_vertex = true;
                        type = inner_event(a,i).type;
                        %y_current = inner_event(a,i).location(2);
                    elseif (inner_event(a,i).location2(2)<=y1 && inner_event(a,i).location2(2)>=y2)
                        x_search  = inner_event(a, i).location2(1);
                        nr1 = a;
                        nr2 = i;
                        outer = false;
                        first_vertex = false;
                        type = inner_event(a,i).type;
                        %y_current = inner_event(a,i).location(2);
                    end
                end
            end  
        end
    end
end
function [y_res] = calculateVertex(x, vertex1, vertex2)
x_delta = x-vertex1(1);
vector = vertex2-vertex1;
frac = x_delta/vector(1);
y_res = vertex1(2)+vector(2)*frac;
end
function [outer_event, outer_vertex, inner_event, inner_vertex] = createEvents(x_out, y_out, x_hole, y_hole)
    for i = 1:size(x_out, 2)
       %Use orientation here in cgal c++ code
       %Start with leftmost vertex & do %size
       outer_vertex(i).location= [x_out(i), y_out(i)];
       %if clockwise
            if (i<size(x_out,2)) %In c++: i<size(x_out) ? next_vertex = i+1 : next_vertex = 1
                next_vertex = i+1;
            else
                next_vertex = 1;
            end
            if (i>1)
                before_vertex = i-1;
            else
                before_vertex = size(x_out,2);
            end
            outer_vertex(i).ceiling = next_vertex;
            outer_vertex(i).floor = before_vertex;
            outer_vertex(i).merge_next = false;
       if(y_out(next_vertex) > y_out(before_vertex))
           outer_vertex(i).side = 1;
           if (x_out(next_vertex)>=x_out(i))
               outer_vertex(i).colour1 = 0;
           else 
               outer_vertex(i).colour1 = 1;
           end
           if (x_out(next_vertex)==x_out(i))
               outer_vertex(i).merge_next = true;
           end
           if (x_out(before_vertex)>=x_out(i))
               outer_vertex(i).colour2 = 1;
           else 
               outer_vertex(i).colour2 = 0;
           end
       else
           outer_vertex(i).side = 0;
           if (x_out(before_vertex)>=x_out(i))
               outer_vertex(i).colour1 = 1;
           else 
               outer_vertex(i).colour1 = 0;
           end
           if (x_out(next_vertex)==x_out(i))
               outer_vertex(i).merge_next = true;
           end
           if (x_out(next_vertex)>x_out(i))
               outer_vertex(i).colour2 = 0;
           else 
               outer_vertex(i).colour2 = 1;
           end
       end
    end
    a=1;
    for i=1:size(outer_vertex,2)
        if ((i>1 && outer_vertex(i-1).merge_next==0)||(i==1 && outer_vertex(size(outer_vertex,2)).merge_next==0) )
            if (outer_vertex(i).merge_next)
                outer_event(a).location = outer_vertex(i).location;
                outer_event(a).side = outer_vertex(i).side;
                outer_event(a).location2 = outer_vertex(mod(i,size(outer_vertex,2))+1).location;
                outer_event(a).closed = false;
                outer_event(a).upper = false;
                outer_event(a).lower = false;
                outer_event(a).ceiling = outer_vertex(mod(i,size(outer_vertex,2))+1).ceiling;
                outer_event(a).floor = outer_vertex(i).floor;
                if (outer_vertex(i).colour1 == 0 && outer_vertex(mod(i,size(outer_vertex,2))+1).colour2 == 1) 
                    outer_event(a).type = 1;
                elseif(outer_vertex(i).colour1 == 1 && outer_vertex(mod(i,size(outer_vertex,2))+1).colour2 == 0) 
                    outer_event(a).type = 0;
                else
                    outer_event(a).type = 2;
                end
            else 
                outer_event(a).location = outer_vertex(i).location;
                outer_event(a).location2 = outer_vertex(i).location;
                outer_event(a).side = outer_vertex(i).side;
                outer_event(a).ceiling = outer_vertex(i).ceiling;
                outer_event(a).floor = outer_vertex(i).floor;
                outer_event(a).closed = false;
                outer_event(a).upper = false;
                outer_event(a).lower = false;
                if (outer_vertex(i).side == 1)
                    if (outer_vertex(i).colour1 == 0 && outer_vertex(i).colour2 == 1) 
                        outer_event(a).type = 0;
                    elseif (outer_vertex(i).colour1 == 1 && outer_vertex(i).colour2 == 0)
                        outer_event(a).type = 1;
                    else 
                        outer_event(a).type = 2;
                    end
                else
                    if (outer_vertex(i).colour1 == 0 && outer_vertex(i).colour2 == 1) 
                        outer_event(a).type = 1;
                    elseif (outer_vertex(i).colour1 == 1 && outer_vertex(i).colour2 == 0)
                        outer_event(a).type = 0;
                    else 
                        outer_event(a).type = 2;
                    end
                end
            end
            a=a+1;
        end
    end
    for a = 1:size(x_hole, 1)
        n = size(x_hole(a, :), 2);
        while (isnan(x_hole(a, n)))
            n=n-1;
        end
        for i = 1:n
           inner_vertex(a,i).location= [x_hole(a,i), y_hole(a,i)];
           %if c.clockwise
                if (i>1) %In c++: i<size(x_out) ? next_vertex = i+1 : next_vertex = 1
                    next_vertex = i-1;
                else
                    next_vertex = n;
                end
                if (i<n)
                    before_vertex = i+1;
                else
                    before_vertex = 1;
                end
                inner_vertex(a,i).ceiling = next_vertex;
                inner_vertex(a,i).floor = before_vertex;
                inner_vertex(a,i).merge_next = false;
           if(y_hole(a,next_vertex) > y_hole(a,before_vertex))
               inner_vertex(a,i).side = 1;
               if (x_hole(a,next_vertex)>x_hole(a,i))
                   inner_vertex(a,i).colour1 = 1;
               else 
                   inner_vertex(a,i).colour1 = 0;
               end
               if (x_hole(a,next_vertex)==x_hole(a,i))
                   inner_vertex(a,i).merge_next = true;
               end
               if (x_hole(a,before_vertex)>x_hole(a,i))
                   inner_vertex(a,i).colour2 = 0;
               else 
                   inner_vertex(a,i).colour2 = 1;
               end
           else
               inner_vertex(a,i).side = 0;
               if (x_hole(a,before_vertex)>x_hole(a,i))
                   inner_vertex(a,i).colour1 = 0;
               else 
                   inner_vertex(a,i).colour1 = 1;
               end
               if (x_hole(a,next_vertex)==x_hole(a,i))
                   inner_vertex(a,i).merge_next = true;
               end
               if (x_hole(a,next_vertex)>x_hole(a,i))
                   inner_vertex(a,i).colour2 = 1;
               else 
                   inner_vertex(a,i).colour2 = 0;
               end
           end
        end
        x=1;
        n = size(inner_vertex,2);
        while (isnan(x_hole(a, n)))
            n=n-1;
        end
        for i=1:n
            %if c.clockwise
            if ((i<n && ~inner_vertex(a,i+1).merge_next)||(i==n && ~inner_vertex(a,1).merge_next))
                if (inner_vertex(a, i).merge_next)
                    inner_event(a, x).location = inner_vertex(a, i).location;
                    inner_event(a, x).side = inner_vertex(a, i).side;
                    if (i == 1)
                        m=n;
                    else
                        m=i-1;
                    end
                    inner_event(a, x).location2 = inner_vertex(a, m).location;
                    inner_event(a, x).closed = false;
                    inner_event(a, x).upper = false;
                    inner_event(a, x).lower = false;
                    if (inner_vertex(a, i).side == 1)
                        if (inner_vertex(a,i).colour2 == 0 && inner_vertex(a, m).colour1 == 1) 
                            inner_event(a,x).type = 0;
                        elseif (inner_vertex(a,i).colour2 == 1 && inner_vertex(a, m).colour1 == 0)
                            inner_event(a,x).type = 1;
                        else 
                            inner_event(a,x).type = 2;
                        end
                    else
                        if (inner_vertex(a,i).colour1 == 0 && inner_vertex(a, m).colour2 == 1) 
                            inner_event(a,x).type = 0;
                        elseif (inner_vertex(a,i).colour1 == 1 && inner_vertex(a, m).colour2 == 0)
                            inner_event(a,x).type = 1;
                        else 
                            inner_event(a,x).type = 2;
                        end
                    end
                    inner_event(a, x).floor = inner_vertex(a,i).floor;
                    inner_event(a, x).ceiling = inner_vertex(a,m).ceiling;
                else 
                    inner_event(a,x).location = inner_vertex(a, i).location;
                    inner_event(a,x).location2 = inner_vertex(a, i).location;
                    inner_event(a,x).side = inner_vertex(a, i).side;
                    inner_event(a,x).ceiling = inner_vertex(a, i).ceiling;
                    inner_event(a,x).floor = inner_vertex(a, i).floor;
                    inner_event(a,x).closed = false;
                    inner_event(a,x).upper = false;
                    inner_event(a,x).lower = false;
                    if (inner_vertex(a, i).side == 1)
                        if (inner_vertex(a, i).colour1 == 0 && inner_vertex(a, i).colour2 == 1) 
                            inner_event(a, x).type = 1;
                        elseif (inner_vertex(a,i).colour1 == 1 && inner_vertex(a,i).colour2 == 0)
                            inner_event(a, x).type = 0;
                        else 
                            inner_event(a, x).type = 2;
                        end
                    else
                        if (inner_vertex(a,i).colour1 == 0 && inner_vertex(a,i).colour2 == 1) 
                            inner_event(a,x).type = 0;
                        elseif (inner_vertex(a,i).colour1 == 1 && inner_vertex(a,i).colour2 == 0)
                            inner_event(a,x).type = 1;
                        else 
                            inner_event(a,x).type = 2;
                        end
                    end
                end
                x=x+1;
            end
        end
    end
end
function [edge_list, edge_upper_number, edge_lower_number, edge_upper, edge_lower, inner_event, outer_event] = getEdges(edge_list, inner_event, outer_event, inner_vertex, outer_vertex)
    x_value = inf;
    y_value = inf;
    y_value2 = inf;
    for i=size(edge_list,1):-1:1
        if (edge_list(i,1)<x_value)
            edge_upper_number = i;
            edge_lower_number = i;
            x_value = edge_list(i,1);
            y_value = edge_list(i,2);
            y_value2 = edge_list(i,4);
        elseif(edge_list(i,1)==x_value)
            if(edge_upper_number==edge_lower_number||(abs(edge_list(i,2)-y_value)<abs(edge_list(edge_lower_number,2)-edge_list(edge_upper_number,2))||(abs(edge_list(i,4)-y_value2)<abs(edge_list(edge_lower_number,4)-edge_list(edge_upper_number,4)))))
                if ((edge_list(i,2)<y_value||(edge_list(i,2)==y_value&&edge_list(i,4)<y_value2)))
                        edge_lower_number = i;
                else
                        edge_upper_number = i;
                end
            end
        end
    end
    [event_nr, event_nr2, outer] = find_next_vertex(-inf, inner_event, outer_event, 0, 0, true);
    if(~outer)
        x_now = inner_event(event_nr, event_nr2).location(1);
        if (inner_event(event_nr, event_nr2).type == 0 && (x_value > x_now))
            if ((inner_event(event_nr, event_nr2).location2(2))~=(inner_event(event_nr, event_nr2).location(2)))
                edge_list = [edge_list; inner_event(event_nr, event_nr2).location inner_vertex(event_nr, inner_event(event_nr, event_nr2).floor).location; inner_event(event_nr, event_nr2).location2 inner_vertex(event_nr, inner_event(event_nr, event_nr2).ceiling).location];
                edge_upper_number = size(edge_list,1)-1;
                edge_lower_number = size(edge_list,1);
            else
                edge_list = [edge_list; inner_event(event_nr, event_nr2).location inner_vertex(event_nr, inner_event(event_nr, event_nr2).floor).location; inner_event(event_nr, event_nr2).location inner_vertex(event_nr, inner_event(event_nr, event_nr2).ceiling).location];
                edge_upper_number = size(edge_list,1)-1;
                edge_lower_number = size(edge_list,1);
            end
            inner_event(event_nr, event_nr2).closed = true;
        elseif (x_value == x_now && edge_list(edge_upper_number, 2) > inner_event(event_nr, event_nr2).location(2) && edge_list(edge_lower_number, 2) < inner_event(event_nr, event_nr2).location(2))
            if ((inner_event(event_nr, event_nr2).location2(2))~=(inner_event(event_nr, event_nr2).location(2)))
                edge_list = [edge_list; inner_event(event_nr, event_nr2).location inner_vertex(event_nr, inner_event(event_nr, event_nr2).floor).location; inner_event(event_nr, event_nr2).location2 inner_vertex(event_nr, inner_event(event_nr, event_nr2).ceiling).location];
                edge_lower_number = size(edge_list,1);
            else
                edge_list = [edge_list; inner_event(event_nr, event_nr2).location inner_vertex(event_nr, inner_event(event_nr, event_nr2).floor).location; inner_event(event_nr, event_nr2).location inner_vertex(event_nr, inner_event(event_nr, event_nr2).ceiling).location];
                edge_lower_number = size(edge_list,1);
            end
            inner_event(event_nr, event_nr2).closed = true;
        end
    else
        x_now = outer_event(event_nr).location(1);
        if (outer_event(event_nr).type == 0 && x_value > x_now)
            if ((outer_event(event_nr).location2(2))~=(outer_event(event_nr).location(2)))
                edge_list = [edge_list; outer_event(event_nr).location outer_vertex(outer_event(event_nr).floor).location; outer_event(event_nr).location2 outer_vertex(outer_event(event_nr).ceiling).location];
                edge_upper_number = size(edge_list,1);
                edge_lower_number = size(edge_list,1)-1;
            else
                edge_list = [edge_list; outer_event(event_nr).location outer_vertex(outer_event(event_nr).floor).location; outer_event(event_nr).location outer_vertex(outer_event(event_nr).ceiling).location];
                edge_upper_number = size(edge_list,1);
                edge_lower_number = size(edge_list,1)-1;
            end
            outer_event(event_nr).closed = true;
        end
    end
    edge_upper=edge_list(edge_upper_number,:);
    edge_lower=edge_list(edge_lower_number,:);
end
function [polygon_nr, vertex_nr, created_polygons] = createNewPolygon(polygon_nr, created_polygons, edge_upper, edge_lower)
    polygon_nr = (polygon_nr)+1;
    vertex_nr= 1;

    created_polygons(polygon_nr, vertex_nr).point = edge_upper(1:2);
    created_polygons(polygon_nr, vertex_nr).direction = edge_upper(3:4);
    vertex_nr = vertex_nr+1;

    created_polygons(polygon_nr, vertex_nr).point = edge_lower(1:2);
    created_polygons(polygon_nr, vertex_nr).direction = edge_lower(3:4);
    vertex_nr = vertex_nr+1;
end
function drawPolygons(created_polygons)
    hold on
    for i = 1:size(created_polygons,1)
        for a = 1:size(created_polygons,2)
            if (isempty(created_polygons(i,a).point))
              a=a-1;
              break
            end
             x_polygon(i,a) = created_polygons(i,a).point(1);
             y_polygon(i,a) = created_polygons(i,a).point(2);
        end
        pgon_end = polyshape({x_polygon(i,1:a)},{y_polygon(i,1:a)});
        plot(pgon_end)
    end
end
function [x_current_next, upper_vertices, upper_number, created_polygons, vertex_nr, edge_list, inner_event] = createInnerVertices(polygon_nr, first_vertex, inner_event, inner_vertex, event_nr, event_nr2, upper_number, upper_vertices, created_polygons, vertex_nr, edge_list)
    x_current_next = inner_event(event_nr, event_nr2).location(1);
    if (inner_event(event_nr, event_nr2).type == 2)
        if (first_vertex)
            new_point = inner_event(event_nr, event_nr2).location;
        else
            new_point = inner_event(event_nr, event_nr2).location2;
        end
        if (upper_number == 1)
            upper_point = calculateVertex(x_current_next, created_polygons(polygon_nr, 1).point, created_polygons(polygon_nr, 1).direction);
        else
            upper_point = calculateVertex(x_current_next, upper_vertices(upper_number-1).point, upper_vertices(upper_number-1).direction);
        end
        if (upper_point == new_point(2))
            upper_vertices(upper_number).point = new_point;
            upper_vertices(upper_number).direction = inner_vertex(event_nr, inner_event(event_nr, event_nr2).floor).location;
            upper_number = upper_number+1;
            location = inner_event(event_nr, event_nr2).location;
            if ((inner_event(event_nr, event_nr2).location2(2))~=(inner_event(event_nr, event_nr2).location(2)))
                upper_vertices(upper_number).point = inner_event(event_nr, event_nr2).location2;
                upper_vertices(upper_number).direction = inner_vertex(event_nr, inner_event(event_nr, event_nr2).floor).location;
                upper_number = upper_number+1;
                location = inner_event(event_nr, event_nr2).location2;
            end
            [edge_list, inner_event(event_nr, event_nr2)] = updateEdgeList(edge_list, inner_event(event_nr, event_nr2), location, inner_vertex(event_nr, inner_event(event_nr, event_nr2).floor).location, false);
        else
            created_polygons(polygon_nr, vertex_nr).point = new_point;
            created_polygons(polygon_nr, vertex_nr).direction = inner_vertex(event_nr, inner_event(event_nr, event_nr2).ceiling).location;
            vertex_nr = vertex_nr+1;
            location = inner_event(event_nr, event_nr2).location;
            if ((inner_event(event_nr, event_nr2).location2(2))~=(inner_event(event_nr, event_nr2).location(2)))
                created_polygons(polygon_nr, vertex_nr).point = inner_event(event_nr, event_nr2).location2;
                created_polygons(polygon_nr, vertex_nr).direction = inner_vertex(event_nr, inner_event(event_nr, event_nr2).ceiling).location;
                vertex_nr = vertex_nr+1;
                location = inner_event(event_nr, event_nr2).location2;
            end
            [edge_list, inner_event(event_nr, event_nr2)] = updateEdgeList(edge_list, inner_event(event_nr, event_nr2), location, inner_vertex(event_nr, inner_event(event_nr, event_nr2).ceiling).location, false);
        end
    end
end
function [created_polygons, edge_list, inner_event]=innerPolygonEnd(x_current_next,first_vertex, outer_event, inner_event, event_nr, event_nr2, created_polygons, polygon_nr, upper_vertices, upper_number, edge_list, edge_upper_number, edge_lower_number, vertex_nr)    
    if (first_vertex)
        next_point = inner_event(event_nr, event_nr2).location;
    else
        next_point = inner_event(event_nr, event_nr2).location2;
    end
    if (upper_number == 1)
        upper_vertex = created_polygons(polygon_nr, 1);
    else
        upper_vertex = upper_vertices(upper_number-1);
    end
    if (next_point(2) ~= calculateVertex(x_current_next, upper_vertex.point, upper_vertex.direction))
        created_polygons(polygon_nr, vertex_nr).point = next_point;
        vertex_nr = vertex_nr+1;
        created_polygons(polygon_nr, vertex_nr).point = [x_current_next,calculateVertex(x_current_next, upper_vertex.point, upper_vertex.direction)]; 
        if (edge_list(edge_upper_number, 3) ~= created_polygons(polygon_nr, vertex_nr).point(1) || edge_list(edge_upper_number, 4) ~= created_polygons(polygon_nr, vertex_nr).point(2))
            edge_list(edge_upper_number, 1:2) = created_polygons(polygon_nr, vertex_nr).point;
        else 
           edge_list = [edge_list(1:edge_upper_number-1, :);edge_list(edge_upper_number+1:size(edge_list,1), :)];
           [event_nr_new, event_nr2_new, outer, first_vertex] = find_next_vertex(created_polygons(polygon_nr, vertex_nr).point(1), inner_event, outer_event, upper_vertex, upper_vertex, false);
           inner_event(event_nr_new, event_nr2_new).lower = true;
           if (inner_event(event_nr_new, event_nr2_new).upper && inner_event(event_nr_new, event_nr2_new).lower)
                inner_event(event_nr_new, event_nr2_new).closed = true;
           end
           if (edge_lower_number > edge_upper_number)
               edge_lower_number = edge_lower_number - 1;
           end
        end
        edge_list = [edge_list(1:edge_lower_number-1, :);edge_list(edge_lower_number+1:size(edge_list,1), :)];
        inner_event(event_nr, event_nr2).upper = true;
    else 
        created_polygons(polygon_nr, vertex_nr).point = [x_current_next,calculateVertex(x_current_next, created_polygons(polygon_nr, vertex_nr-1).point, created_polygons(polygon_nr, vertex_nr-1).direction)];
        vertex_nr = vertex_nr+1;
        created_polygons(polygon_nr, vertex_nr).point = next_point;
        if (edge_list(edge_lower_number, 3) ~= created_polygons(polygon_nr, vertex_nr-1).point(1) || edge_list(edge_lower_number, 4) ~= created_polygons(polygon_nr, vertex_nr-1).point(2))
            edge_list(edge_lower_number, 1:2) = created_polygons(polygon_nr, vertex_nr-1).point;
        else 
           edge_list = [edge_list(1:edge_lower_number-1, :);edge_list(edge_lower_number+1:size(edge_list,1), :)];
           [event_nr_new, event_nr2_new, outer, first_vertex] = find_next_vertex(created_polygons(polygon_nr, vertex_nr-1).point(1), inner_event, outer_event, created_polygons(polygon_nr, 2), created_polygons(polygon_nr, 2), false);
           inner_event(event_nr_new, event_nr2_new).upper = true;
           if (inner_event(event_nr_new, event_nr2_new).upper&&inner_event(event_nr_new, event_nr2_new).lower)
                inner_event(event_nr_new, event_nr2_new).closed = true;
           end
            if (edge_upper_number > edge_lower_number)
               edge_upper_number = edge_upper_number - 1;
           end
        end
        edge_list = [edge_list(1:edge_upper_number-1, :);edge_list(edge_upper_number+1:size(edge_list,1), :)];
        inner_event(event_nr, event_nr2).lower = true;
    end
    if (inner_event(event_nr, event_nr2).upper&&inner_event(event_nr, event_nr2).lower)
        inner_event(event_nr, event_nr2).closed = true;
    end
    vertex_nr = vertex_nr+1;
    for m = size(upper_vertices,2):-1:1
        created_polygons(polygon_nr, vertex_nr) = upper_vertices(m);
        vertex_nr = vertex_nr+1;
    end
end
function [edge_list, created_polygons]= newObstacle(created_polygons, polygon_nr, x_current_next, edge_list, edge_lower_number, edge_upper_number, vertex_nr, upper_vertices, event, direction1, direction2)
    %Close previous polygon
    [created_polygons, edge_list] = closePolygon(x_current_next, edge_list, created_polygons, polygon_nr, vertex_nr, upper_vertices, edge_lower_number, edge_upper_number);
    edge_list = addNewEdge(edge_list, event, direction1, direction2);
end
function edge_list = addNewEdge(edge_list, event, direction1, direction2)
    %Open new polygon
    if ((event.location2(2))~=(event.location(2)))
        edge_list = [edge_list(1,:);event.location2 direction1;edge_list(2:size(edge_list, 1),:)];
        edge_list = [edge_list(1,:);event.location direction2 ;edge_list(2:size(edge_list, 1),:)];
    else
        edge_list = [edge_list(1,:);event.location direction1;edge_list(2:size(edge_list, 1),:)];
        edge_list = [edge_list(1,:);event.location direction2;edge_list(2:size(edge_list, 1),:)];
    end
end
function [upper_vertices, upper_number, created_polygons, vertex_nr, edge_list, outer_event] = createOuterVertices(polygon_nr, outer_event, event_nr, upper_number, upper_vertices, created_polygons, vertex_nr, edge_list, outer_vertex)
    x_current_next = outer_event(event_nr).location(1);    
    if (upper_number == 1)
        upper_point = calculateVertex(x_current_next, created_polygons(polygon_nr, 1).point, created_polygons(polygon_nr, 1).direction);
    else
        upper_point = calculateVertex(x_current_next, upper_vertices(upper_number-1).point, upper_vertices(upper_number-1).direction);
    end
    if (upper_point == outer_event(event_nr).location(2))
        upper_vertices(upper_number).point = outer_event(event_nr).location;
        upper_vertices(upper_number).direction = outer_vertex(outer_event(event_nr).ceiling).location;
        upper_number = upper_number+1;
        if ((outer_event(event_nr).location2(2))~=(outer_event(event_nr).location(2)))
            upper_vertices(upper_number).point = outer_event(event_nr).location2;
            upper_vertices(upper_number).direction = outer_vertex(outer_event(event_nr).ceiling).location;
            upper_number = upper_number+1;
        end
        if (outer_event(event_nr).location(2)~=outer_event(event_nr).location2(2))
            location = outer_event(event_nr).location2;
        else
            location = outer_event(event_nr).location;
        end
        [edge_list, outer_event(event_nr)] = updateEdgeList(edge_list, outer_event(event_nr), location, outer_vertex(outer_event(event_nr).ceiling).location, false);
    else
        if ((outer_event(event_nr).location2(2))~=(outer_event(event_nr).location(2)))
            created_polygons(polygon_nr, vertex_nr).point = outer_event(event_nr).location2;
            created_polygons(polygon_nr, vertex_nr).direction = outer_vertex(outer_event(event_nr).floor).location;
            vertex_nr = vertex_nr+1;
        end
        created_polygons(polygon_nr, vertex_nr).point = outer_event(event_nr).location;
        created_polygons(polygon_nr, vertex_nr).direction = outer_vertex(outer_event(event_nr).floor).location;
        vertex_nr = vertex_nr+1;
        [edge_list, outer_event(event_nr)] = updateEdgeList(edge_list, outer_event(event_nr), outer_event(event_nr).location, outer_vertex(outer_event(event_nr).floor).location, false);
    end
end
function [created_polygons, edge_list] = closePolygon(x_current, edge_list, created_polygons, polygon_nr, vertex_nr, upper_vertices, edge_lower_number, edge_upper_number)
    %Close previous polygon
    created_polygons(polygon_nr, vertex_nr).point = [x_current,calculateVertex(x_current, created_polygons(polygon_nr, vertex_nr-1).point, created_polygons(polygon_nr, vertex_nr-1).direction)]; 
    created_polygons(polygon_nr, vertex_nr).direction = created_polygons(polygon_nr, vertex_nr-1).direction;
    if (edge_list(edge_lower_number, 3) == created_polygons(polygon_nr, vertex_nr).point(1) && edge_list(edge_lower_number, 4) == created_polygons(polygon_nr, vertex_nr).point(2))
        edge_list(edge_lower_number, :) = [];
        if (edge_upper_number > edge_lower_number)
            edge_upper_number = edge_upper_number-1;
        end
    else    
        edge_list(edge_lower_number, 1:2) = created_polygons(polygon_nr, vertex_nr).point;
    end
    vertex_nr = vertex_nr+1;
    if (size(upper_vertices, 2) < 1)
        created_polygons(polygon_nr, vertex_nr).point = [x_current,calculateVertex(x_current, created_polygons(polygon_nr, 1).point, created_polygons(polygon_nr, 1).direction)]; 
        created_polygons(polygon_nr, vertex_nr).direction = created_polygons(polygon_nr, 1).direction;
    else
        created_polygons(polygon_nr, vertex_nr).point = [x_current,calculateVertex(x_current, upper_vertices(size(upper_vertices, 2)).point, upper_vertices(size(upper_vertices, 2)).direction)]; 
        created_polygons(polygon_nr, vertex_nr).direction = upper_vertices(size(upper_vertices, 2)).direction;
    end
    
    
    if (edge_list(edge_upper_number, 3) == created_polygons(polygon_nr, vertex_nr).point(1) && edge_list(edge_upper_number, 4) == created_polygons(polygon_nr, vertex_nr).point(2))
        edge_list(edge_upper_number, :) = [];
    else    
        edge_list(edge_upper_number, 1:2) = created_polygons(polygon_nr, vertex_nr).point;
    end
    
    vertex_nr = vertex_nr+1;
    for m = size(upper_vertices,2):-1:1
        created_polygons(polygon_nr, vertex_nr) = upper_vertices(m);
        vertex_nr = vertex_nr+1;
    end
end
function [edge_list, event] = updateEdgeList(edge_list, event, location, direction, clear)
    i = 1;
    while (i <= size(edge_list,1))
        if((edge_list(i,3) == event.location(1) && edge_list(i,4) == event.location(2)) || (edge_list(i,3) == event.location2(1) && edge_list(i,4) == event.location2(2)))
            if (clear)
                edge_list(i, :) = [];
                if (event.upper)
                    event.closed = true;
                else
                    event.upper = true;
                end
            else 
                edge_list(i, :) = [location direction];
                event.closed = true;
                break;
            end
        else
            i = i+1;
        end
    end
end