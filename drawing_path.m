function drawing_path(ref_poses,curr_poses,target_index)
    cx = ref_poses(:,1);
    cy = ref_poses(:,2);
    while length(cx)>target_index
        x = curr_poses(1,1);
        y = curr_poses(1,2);
        % pause(0.1)
        plot(cx,cy,'g',x,y,'r-*')                 %��Ԥ���λ������ɫ��ʾ����ǰλ���ú�ɫ��ʾ
        drawnow
        hold on
  
    end
end