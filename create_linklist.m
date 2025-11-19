function linkList = create_linklist()
    linkList = [];
    % append i to linklist
    for i = 1:3
        a_i = dhTable(i, 'a');
        d_i = dhTable(i, 'd');
        alpha_i = dhTable(i, 'alpha');
        link_i = createLink( a_i, d_i, alpha_i, [], 0, 0 ,...
            0, 0 );
       linkList = [linkList, link_i]; % Append the created link to the linkList
    end
end