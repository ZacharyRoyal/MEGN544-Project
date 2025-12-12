function linkList = create_linklist()
    linkList = [];
    % append i to linklist
    for i = 1:4
        a_i = dhTable(i, 'a');
        d_i = dhTable(i, 'd');
        alpha_i = dhTable(i, 'alpha');

        density = 2700;
        mass = density * link_len;  % Mass for links 1, 2, 3

        if i == 4   % It's a laser...no mass or inertia
            link_i = createLink(a_i, [], alpha_i, 0, 0, 0, 0, 0);
        else    % Inertia tensor calculations
            if a_i == 0 % Links 1 and 2: hollow cylinder along z
                Ixx = (1/12) * m * (3*radius^2 + link_len^2);
                Iyy = Ixx;
                Izz = m * radius^2;
                inertia_tensor = diag([Ixx, Iyy, Izz]);
                cent_of_mass = [0; 0; link_len/2];
            else    % Link 3 along x
                Ixx = m * radius^2;
                Iyy = (1/12) * m * (3*radius^2 + link_len^2);
                Izz = Iyy;
                inertia_tensor = diag([Ixx, Iyy, Izz]);
                cent_of_mass = [link_len/2; 0; 0];
            end
            link_i = createLink( a_i, d_i, alpha_i, [], 0, cent_of_mass, mass, inertia_tensor );
        end
        
        linkList = [linkList, link_i];
    end
end