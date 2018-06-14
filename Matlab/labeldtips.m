function output_txt = labeldtips(obj,event_obj,X,Y,task,theta,psi)

pos = get(event_obj,'Position');
x = pos(1); y = pos(2);
output_txt = {['Position: (',num2str(pos(1),3),', ',num2str(pos(2),3),')']};

if event_obj.Target.Marker == 'x'
    output_txt{end+1} = 'Object: Task';
else
    output_txt{end+1} = 'Object: Robot';
end

if event_obj.Target.Color == [1 0 0]
    output_txt{end+1} = 'Type: Red';
elseif event_obj.Target.Color == [0 1 0]
    output_txt{end+1} = 'Type: Green';
elseif event_obj.Target.Color == [0 0 1]
    output_txt{end+1} = 'Type: Blue';
else
    output_txt{end+1} = 'Type: Else';
end

if event_obj.Target.Marker == 'x'
    ind = find(task == x);
    output_txt{end+1} = ['Requirements: (',...
        num2str(psi(1,ind)),', ',...
        num2str(psi(2,ind)),', ',...
        num2str(psi(3,ind)),')'];
else
    indx = find(X == x);
    ind = [mod(indx,size(X,1)) floor(indx/size(X,1))+1];
    output_txt{end+1} = ['Attributes: (',...
        num2str(theta(1,ind(2))),', ',...
        num2str(theta(2,ind(2))),', ',...
        num2str(theta(3,ind(2))),')'];
end

end