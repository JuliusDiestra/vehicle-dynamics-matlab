function dd_q = solveddq(q,dq,T,V,Q)
    L = T - V;
    L_q = jacobian(L,q)';
    L_dq = jacobian(L,dq)';
    dd_q = (jacobian(L_dq,dq))\(-jacobian(L_dq,q)*dq +L_q+Q);
    dd_q = simplify(dd_q);
end