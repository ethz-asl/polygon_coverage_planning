clear all
close all 
x_out = [-8 4 4 11 11 10 10 20 20 10 -5 -5 -10 -10 -7 -7]; %clockwise
y_out = [10 10 8 10 8 3 1 0 -1 -12 -11 -12 -12 -11 0 1];
%number of holes has to be same
x_hole(1, :) = [ -4 -8, 0 0 4 3 3 4 -1 -1 -5 -4]; %c.clockwise
y_hole(1, :)= [ 1 -8, -1 0 0 1 1.5 3 3 5 5 4];
x_hole(2, 1:4) = [10,  12 12, NaN]; %c.clockwise
y_hole(2, 1:4) = [-5,  -6, -5, NaN];
x_hole(3, 1:5) = [-1, -1, 1, 1 NaN]; %c.clockwise
y_hole(3, 1:5) = [-5, -6, -6, -5 NaN];
x_hole(4, 1:5) = [-7, -7, 4, 4 NaN]; %c.clockwise
y_hole(4, 1:5) = [-9, -10, -10, -9 NaN];
x_hole(5, 1:5) = [3, 3, 4, 4 NaN]; %c.clockwise
y_hole(5, 1:5) = [6, 5, 5, 6 NaN];
pgon = polyshape({x_out, x_hole(1, :), x_hole(2, :), x_hole(3, :), x_hole(4, :), x_hole(5, :)},{y_out, y_hole(1, :), y_hole(2,:), y_hole(3,:), y_hole(4, :), y_hole(5, :)});
plot(pgon)
[outer_event, outer_vertex, inner_event, inner_vertex] = createEvents(x_out, y_out, x_hole, y_hole);
%Start algorithm
polygon_nr = 0;
times = 0;
edge_list = [];
created_polygons = [];

[event_nr, event_nr2, outer, first_vertex] = find_next_vertex(-inf, inner_event, outer_event, 0, 0, true);
%Assuming first vertex is always outside
edge_list(1, :)=[outer_event(event_nr).location outer_event(event_nr).floor];
edge_list(2, :)= [outer_event(event_nr).location2 outer_event(event_nr).ceiling];
outer_event(event_nr).closed = true; 

while (~isempty(edge_list)) %times < 6%
    [edge_list, edge_upper_number, edge_lower_number, edge_upper, edge_lower, inner_event, outer_event] = getEdges(edge_list, inner_event, outer_event);
    [polygon_nr, vertex_nr, created_polygons] = createNewPolygon(polygon_nr, created_polygons, edge_upper, edge_lower); 
    [event_nr, event_nr2, outer, first_vertex] = find_next_vertex(-inf, inner_event, outer_event, created_polygons(polygon_nr, 1), created_polygons(polygon_nr, vertex_nr-1), false);
    if (outer)
        x_current = outer_event(event_nr).location(1);
        type = outer_event(event_nr).type;
    else 
        x_current = inner_event(event_nr, event_nr2).location(1);
        type = inner_event(event_nr, event_nr2).type;
    end
    upper_vertices = [];
    while (type == 2) 
        if (size(upper_vertices,2) < 1)
            [event_nr, event_nr2, outer, first_vertex] = find_next_vertex(x_current, inner_event, outer_event, created_polygons(polygon_nr, 1), created_polygons(polygon_nr, vertex_nr-1), false);
        else
            [event_nr, event_nr2, outer, first_vertex] = find_next_vertex(x_current, inner_event, outer_event, upper_vertices(end), created_polygons(polygon_nr, vertex_nr-1), false);
        end
        if (~outer)
           x_current = inner_event(event_nr, event_nr2).location(1);
           if (inner_event(event_nr, event_nr2).type == 2)
             [upper_vertices, created_polygons, vertex_nr, edge_list, inner_event(event_nr, event_nr2)] = createVertices(polygon_nr, first_vertex, inner_event(event_nr, event_nr2), inner_event(event_nr, event_nr2).floor, inner_event(event_nr, event_nr2).ceiling, upper_vertices, created_polygons, vertex_nr, edge_list, false);
           end
           type = inner_event(event_nr, event_nr2).type;
        else
           x_current = outer_event(event_nr).location(1);
           if (outer_event(event_nr).type == 2)
               [upper_vertices, created_polygons, vertex_nr, edge_list, outer_event(event_nr)] = createVertices(polygon_nr, first_vertex, outer_event(event_nr), outer_event(event_nr).ceiling, outer_event(event_nr).floor, upper_vertices, created_polygons, vertex_nr, edge_list, true);
           end
           type = outer_event(event_nr).type;
        end
    end
    if (~outer)
        x_current = inner_event(event_nr, event_nr2).location(1);
        if (inner_event(event_nr, event_nr2).type == 1)
            [created_polygons, edge_list, inner_event]=innerPolygonEnd(x_current,first_vertex, inner_event, event_nr, event_nr2, created_polygons, polygon_nr, upper_vertices, edge_list, edge_upper_number, edge_lower_number, vertex_nr);
        elseif (inner_event(event_nr, event_nr2).type == 0)
            [created_polygons, edge_list] = closePolygon(x_current, edge_list, created_polygons, polygon_nr, vertex_nr, upper_vertices, edge_lower_number, edge_upper_number);
            edge_list = [inner_event(event_nr, event_nr2).location inner_event(event_nr, event_nr2).floor; inner_event(event_nr, event_nr2).location2 inner_event(event_nr, event_nr2).ceiling; edge_list];
            inner_event(event_nr, event_nr2).closed = true;
        end
    else
        if(outer_event(event_nr).type == 0)
            x_current = outer_event(event_nr).location(1);
            [created_polygons, edge_list] = closePolygon(x_current, edge_list, created_polygons, polygon_nr, vertex_nr, upper_vertices, edge_lower_number, edge_upper_number);
            edge_list = [outer_event(event_nr).location outer_event(event_nr).floor; outer_event(event_nr).location2 outer_event(event_nr).ceiling; edge_list];
            outer_event(event_nr).closed = true;
        elseif(outer_event(event_nr).type == 1)
            x_current = outer_event(event_nr).location(1);
            [created_polygons, edge_list] = closePolygon(x_current, edge_list, created_polygons, polygon_nr, vertex_nr, upper_vertices,edge_lower_number, edge_upper_number);
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
final_polygons = removeDublicatedVeritices(created_polygons);
drawPolygons(final_polygons);
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
                if ((outer_event(i).location(2)<=y1 && outer_event(i).location(2)>=y2) || (outer_event(i).location2(2)<=y1 && outer_event(i).location2(2)>=y2))
                    x_search= outer_event(i).location(1);
                    nr1 = i;
                    outer = true;
                    type = outer_event(i).type;
                    if (outer_event(i).location(2)<=y1 && outer_event(i).location(2)>=y2)
                        first_vertex = true;
                    else
                        first_vertex = false;
                    end
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
                    if ((inner_event(a,i).location(2)<=y1 && inner_event(a,i).location(2)>=y2)||(inner_event(a,i).location2(2)<=y1 && inner_event(a,i).location2(2)>=y2))
                        x_search  = inner_event(a, i).location(1);
                        nr1 =a;
                        nr2 =i;
                        outer = false;
                        type = inner_event(a,i).type;
                        if (inner_event(a,i).location(2)<=y1 && inner_event(a,i).location(2)>=y2)
                            first_vertex = true;
                        else
                            first_vertex = false;
                        end
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
       %if clockwise
        next_vertex = mod(i,size(x_out,2))+1;
        before_vertex = mod(i-2+size(x_out,2),size(x_out,2))+1;
        outer_vertex(i) = initVertex(next_vertex, before_vertex, i, x_out, y_out);
    end
    a=1;
    for i=1:size(outer_vertex,2)
        if ((i>1 && outer_vertex(i-1).merge_next==0)||(i==1 && outer_vertex(size(outer_vertex,2)).merge_next==0) )
            if (outer_vertex(i).merge_next)
                outer_event(a) = initEvent(outer_vertex(i), outer_vertex(mod(i,size(outer_vertex,2))+1));
            else 
                outer_event(a) = initEvent(outer_vertex(i), outer_vertex(i));
            end
            a=a+1;
        end
    end
    for a = 1:size(x_hole, 1)
        n = 1;
        while (~isnan(x_hole(a, n+1)))
            n=n+1;
            if n == size(x_hole, 2) break; end
        end
        for i = 1:n
           %if c.clockwise
            next_vertex = mod(i-2+n,n)+1;
            before_vertex = mod(i,n)+1;
            inner_vertex(a,i) = initVertex(next_vertex, before_vertex, i, x_hole(a,:), y_hole(a,:));
        end
        x=1;
        for i=1:n
            %if c.clockwise
            if ((i<n && ~inner_vertex(a,i+1).merge_next)||(i==n && ~inner_vertex(a,1).merge_next))
                if (inner_vertex(a, i).merge_next)
                    inner_event(a, x) = initEvent(inner_vertex(a, i), inner_vertex(a, mod(i-2+n,n)+1));
                else 
                    inner_event(a, x) = initEvent(inner_vertex(a, i), inner_vertex(a, i));
                end
                x=x+1;
            end
        end
    end
end
function [edge_list, edge_upper_number, edge_lower_number, edge_upper, edge_lower, inner_event, outer_event] = getEdges(edge_list, inner_event, outer_event)
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
            [edge_list,edge_upper_number, edge_lower_number] = findOpenings(inner_event(event_nr, event_nr2), inner_event(event_nr, event_nr2).floor, inner_event(event_nr, event_nr2).ceiling, edge_list, false);
            inner_event(event_nr, event_nr2).closed = true;
        elseif (x_value == x_now && edge_list(edge_upper_number, 2) > inner_event(event_nr, event_nr2).location(2) && edge_list(edge_lower_number, 2) < inner_event(event_nr, event_nr2).location(2))
            edge_list = [edge_list; inner_event(event_nr, event_nr2).location inner_event(event_nr, event_nr2).floor; inner_event(event_nr, event_nr2).location2 inner_event(event_nr, event_nr2).ceiling];
            edge_lower_number = size(edge_list,1);
            inner_event(event_nr, event_nr2).closed = true;
        end
    else
        x_now = outer_event(event_nr).location(1);
        if (outer_event(event_nr).type == 0 && x_value > x_now)
            [edge_list,edge_upper_number, edge_lower_number] = findOpenings(outer_event(event_nr), outer_event(event_nr).floor, outer_event(event_nr).ceiling, edge_list, true);
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
function [upper_vertices, created_polygons, vertex_nr, edge_list, event] = createVertices(polygon_nr, first_vertex, event, direction1, direction2, upper_vertices, created_polygons, vertex_nr, edge_list ,outer)
    x_current = event.location(1);    
    if (~first_vertex && ~outer)
        new_point = event.location2;
        new_point2 = event.location;
    else
        new_point = event.location;
        new_point2 = event.location2;
    end
    if (size(upper_vertices, 2) < 1)
        upper_point = calculateVertex(x_current, created_polygons(polygon_nr, 1).point, created_polygons(polygon_nr, 1).direction);
    else
        upper_point = calculateVertex(x_current, upper_vertices(end).point, upper_vertices(end).direction);
    end
    upper_number = size(upper_vertices, 2);
    if (upper_point == new_point(2))
        upper_vertices(upper_number+1).point = new_point;
        upper_vertices(upper_number+1).direction = direction1;
        if ((event.location2(2))~=(event.location(2)))
            upper_vertices(upper_number+2).point = new_point2;
            upper_vertices(upper_number+2).direction = direction1;
        end
        [edge_list, event] = updateEdgeList(edge_list, event, upper_vertices(end).point, direction1, false);
    else
        if ((event.location2(2))~=(event.location(2)) && outer)
            created_polygons(polygon_nr, vertex_nr).point = event.location2;
            created_polygons(polygon_nr, vertex_nr).direction = direction2;
            vertex_nr = vertex_nr+1;
        end
        created_polygons(polygon_nr, vertex_nr).point = new_point;
        created_polygons(polygon_nr, vertex_nr).direction = direction2;
        vertex_nr = vertex_nr+1;
        if ((event.location2(2))~=(event.location(2)) && ~outer)
            created_polygons(polygon_nr, vertex_nr).point = event.location2;
            created_polygons(polygon_nr, vertex_nr).direction = direction2;
            vertex_nr = vertex_nr+1;
        end
        [edge_list, event] = updateEdgeList(edge_list, event, created_polygons(polygon_nr, vertex_nr-1).point, direction2, false);
    end
end
function [created_polygons, edge_list, inner_event]=innerPolygonEnd(x_current,first_vertex, inner_event, event_nr, event_nr2, created_polygons, polygon_nr, upper_vertices, edge_list, edge_upper_number, edge_lower_number, vertex_nr)    
    edge_list_size = size(edge_list, 1);
    if (first_vertex)
        next_point = inner_event(event_nr, event_nr2).location;
    else
        next_point = inner_event(event_nr, event_nr2).location2;
    end
    if (size(upper_vertices,2)< 1)
        upper_vertex = created_polygons(polygon_nr, 1);
    else
        upper_vertex = upper_vertices(end);
    end
    lower_vertex = created_polygons(polygon_nr, vertex_nr-1);
    [created_polygons, edge_list] = closePolygon(x_current, edge_list, created_polygons, polygon_nr, vertex_nr, upper_vertices, edge_lower_number, edge_upper_number);
    if (next_point(2) ~= calculateVertex(x_current, upper_vertex.point, upper_vertex.direction))
        if (size(edge_list, 1) < edge_list_size - 1)
            inner_event = closeSecondEvent(inner_event, calculateVertex(x_current, upper_vertex.point, upper_vertex.direction), false);
        end
        inner_event(event_nr, event_nr2).upper = true;
    else 
        if (size(edge_list, 1) < edge_list_size - 1)
            inner_event = closeSecondEvent(inner_event, calculateVertex(x_current, lower_vertex.point, lower_vertex.direction), true);
        end
        inner_event(event_nr, event_nr2).lower = true;
    end
    if (inner_event(event_nr, event_nr2).upper&&inner_event(event_nr, event_nr2).lower)
        inner_event(event_nr, event_nr2).closed = true;
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
function final_polygons = removeDublicatedVeritices(created_polygons)
    for i = 1:size(created_polygons,1)
        a = 2;
        final_polygons(i,1).point = created_polygons(i,1).point;
        x = created_polygons(i,1).point(1);
        y = created_polygons(i,1).point(2);
        index = 2;
        while (a <= size(created_polygons,2))
            if (isempty(created_polygons(i,a).point))
              break
            end
            if (x ~= created_polygons(i,a).point(1) || y ~= created_polygons(i,a).point(2))
                final_polygons(i,index).point = created_polygons(i,a).point;
                index = index+1;
            end
            x = created_polygons(i,a).point(1);
            y = created_polygons(i,a).point(2);
            a = a+1;
        end
    end
end
function [edge_list,edge_upper_number, edge_lower_number] = findOpenings(event, direction1, direction2, edge_list, outer)
    edge_list = [edge_list; event.location direction1; event.location2 direction2];
    if (outer)
        edge_upper_number = size(edge_list,1);
        edge_lower_number = size(edge_list,1)-1;
    else
        edge_upper_number = size(edge_list,1)-1;
        edge_lower_number = size(edge_list,1);
    end
end
function vertex = setColours(vertex, x_before, x_after, x_now, side)
   if (x_after >= x_now)  colour1 = 0; else colour1 = 1; end
   if (x_before >= x_now) colour2 = 1; else colour2 = 0; end
    if (side == 1)
       vertex.colour1 = colour1;
       vertex.colour2 = colour2;
    else
        vertex.colour1 = colour2;
        vertex.colour2 = colour1;
    end
end
function event_list = initEvent(vertex_now, vertex_next)
    event_list.location = vertex_now.location;
    event_list.side = vertex_now.side;
    event_list.location2 = vertex_next.location;
    event_list.closed = false;
    event_list.upper = false;
    event_list.lower = false;
    event_list.ceiling = vertex_next.ceiling;
    event_list.floor = vertex_now.floor;
    if (vertex_now.side == 1)
        colour1 = vertex_now.colour2;
        colour2 = vertex_next.colour1;
    else
        colour1 = vertex_now.colour1;
        colour2 = vertex_next.colour2;
    end
    if (colour1 == 0 && colour2 == 1) 
        event_list.type = 1;
    elseif (colour1 == 1 && colour2 == 0)
        event_list.type = 0;
    else 
        event_list.type = 2;
    end
end
function vertex = initVertex(next_vertex, before_vertex, i, x_vertices, y_vertices)
    vertex.location= [x_vertices(i), y_vertices(i)];
    vertex.ceiling = [x_vertices(next_vertex), y_vertices(next_vertex)];
    vertex.floor = [x_vertices(before_vertex), y_vertices(before_vertex)];
    vertex.merge_next = false;
    vertex.colour1 = 0;
    vertex.colour2 = 0;
    if(y_vertices(next_vertex) > y_vertices(before_vertex))
       vertex.side = 1;
       vertex = setColours(vertex, x_vertices(before_vertex), x_vertices(next_vertex), x_vertices(i), 1);
    else
       vertex.side = 0;
       vertex = setColours(vertex, x_vertices(before_vertex), x_vertices(next_vertex), x_vertices(i), 0);
    end
    if (x_vertices(next_vertex)==x_vertices(i))
       vertex.merge_next = true;
    end
end
function inner_event = closeSecondEvent(inner_event, y_event, upper)
    for i = 1:size(inner_event,1) 
       for a =1:size(inner_event(i, :), 2)
            if (isempty(inner_event(i, a).location))
                break;
            end
           if ((inner_event(i,a).location(2) == y_event) || inner_event(i,a).location2(2) == y_event)
                if upper
                    inner_event(i, a).upper = true;
                else
                    inner_event(i, a).lower = true;
                end
               if (inner_event(i, a).upper&&inner_event(i, a).lower)
                    inner_event(i, a).closed = true;
               end
           end
       end
    end
end