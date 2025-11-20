function linkList = create_linklist()
    linkList = [];
    % append i to linklist
    for i = 1:4
        a_i = dhTable(i, 'a');
        d_i = dhTable(i, 'd');
        alpha_i = dhTable(i, 'alpha');
        if i == 4
            link_i = createLink(a_i, [], alpha_i, 0, 0, 0, 0, 0);
        else
            link_i = createLink( a_i, d_i, alpha_i, [], 0, 0 ,...
            0, 0 );
        end
        
       linkList = [linkList, link_i]; % Append the created link to the linkList
    end
end