function c = mass2coriolis(M)
syms q1 q2 q_dot_1 q_dot_2

c = sym(zeros(size(M))); 
q = [q1]; 
q_dot = [q_dot_1];
for i = 1:size(M,1)
    for j = 1:size(M,2)
        for k = 1:size(q,1)
            c(i,j) = c(i,j) + 0.5*(diff(M(i,j),q(k)) + diff(M(i,k),q(j)) - diff(M(k,j),q(i)))*q_dot(k);
        end
    end
end

end 