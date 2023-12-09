function coef_cijk = cijk(bij, bik, bjk, qi, qj, qk)
    coef_cijk = simplify(0.5 * (diff(bij, qk) + diff(bik, qj) - diff(bjk, qi)));
end
