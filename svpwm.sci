function [duty_a_ref, duty_b_ref, duty_c_ref] = svpwm(u_ref_a, u_ref_b, u_ref_c, u_bat, offset, duty_selection)
    if nargin < 5 then
        offset = 50;
    end
    if nargin < 6 then
        duty_selection = %T;
    end
    
    // Avoid division by zero
    u_bat_safe = u_bat;
    u_bat_safe(u_bat == 0) = 1e-6;
    
    u_a = u_ref_a ./ u_bat_safe;
    u_b = u_ref_b ./ u_bat_safe;
    u_c = u_ref_c ./ u_bat_safe;
    
    sector = selector(u_a, u_b, u_c);
    [duty_a, duty_b, duty_c] = duty_offset(offset, u_a, u_b, u_c, sector);
    
    if ~duty_selection then
        duty_a = u_a * 100 + 50;
        duty_b = u_b * 100 + 50;
        duty_c = u_c * 100 + 50;
    end
    
    duty_a_ref = min(max(duty_a, 0), 100);
    duty_b_ref = min(max(duty_b, 0), 100);
    duty_c_ref = min(max(duty_c, 0), 100);
endfunction

function sector = selector(u_a, u_b, u_c)
    sector = ones(u_a);
    
    sector((u_a >= u_b) & (u_c >= u_a)) = 5;
    sector((u_a >= u_b) & (u_c < u_a) & (u_c >= u_b)) = 6;
    sector((u_a >= u_b) & (u_c < u_b)) = 1;
    sector((u_a < u_b) & (u_a >= u_c)) = 2;
    sector((u_a < u_b) & (u_a < u_c) & (u_b >= u_c)) = 3;
    sector((u_a < u_b) & (u_b < u_c)) = 4;
endfunction

function [duty_a, duty_b, duty_c] = duty_offset(offset, u_a, u_b, u_c, sector)
    duty_a = zeros(u_a);
    duty_b = zeros(u_b);
    duty_c = zeros(u_c);
    
    mask1 = (sector == 1) | (sector == 4);
    mask2 = (sector == 2) | (sector == 5);
    mask3 = (sector == 3) | (sector == 6);
    
    duty_a(mask1) = offset + offset .* (u_a(mask1) - u_c(mask1));
    duty_b(mask1) = offset + offset .* (2 * u_b(mask1) - u_a(mask1) - u_c(mask1));
    duty_c(mask1) = offset + offset .* (u_c(mask1) - u_a(mask1));
    
    duty_a(mask2) = offset + offset .* (2 * u_a(mask2) - u_b(mask2) - u_c(mask2));
    duty_b(mask2) = offset + offset .* (u_b(mask2) - u_c(mask2));
    duty_c(mask2) = offset + offset .* (u_c(mask2) - u_b(mask2));
    
    duty_a(mask3) = offset + offset .* (u_a(mask3) - u_b(mask3));
    duty_b(mask3) = offset + offset .* (u_b(mask3) - u_a(mask3));
    duty_c(mask3) = offset + offset .* (2 * u_c(mask3) - u_a(mask3) - u_b(mask3));
endfunction
