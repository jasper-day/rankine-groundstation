from typing import Any

def print_super_amazing_ascii_text_banner() -> None:
    print(
"""
 _____               ______ _               
|  ___|              |  ___| |              
| |____   _____ _ __ | |_  | | _____      __
|  __\ \ / / _ \ '_ \|  _| | |/ _ \ \ /\ / /
| |___\ V /  __/ | | | |   | | (_) \ V  V / 
\____/ \_/ \___|_| |_\_|   |_|\___/ \_/\_/  
                                            
"""
    )

def print_start_notice() -> None:
    print("Endeavour - Rankine EvenFlow Ground Station Backend\n")
    print("Copyright (C) Ollie Killean, James Kitching, Jasper Day - 2025\n")
    print("This program is free software; you can redistribute it and/or")
    print("modify it under the terms of the GNU Lesser General Public")
    print("License as published by the Free Software Foundation; either")
    print("version 3 of the License, or (at your option) any later version.\n")
    print("This program is distributed in the hope that it will be useful,")
    print("but WITHOUT ANY WARRANTY; without even the implied warranty of")
    print("MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU")
    print("Lesser General Public License for more details.\n")

def assert_item_int(elem: str, obj: Any, min = None, max = None) -> None:
    if not isinstance(obj[elem], int):
        raise TypeError(f"Expected {elem} to be an int, got {type(obj[elem])}")
    
    if min is not None and obj[elem] < min:
        raise ValueError(f"Expected {elem} to be greater than {min}, got {obj[elem]}")
    
    if max is not None and obj[elem] > max:
        raise ValueError(f"Expected {elem} to be less than {max}, got {obj[elem]}")

def assert_item_float(elem: str, obj: Any, min = None, max = None) -> None:
    if not isinstance(obj[elem], float):
        raise TypeError(f"Expected {elem} to be a float, got {type(obj[elem])}")
    
    if min is not None and obj[elem] < min:
        raise ValueError(f"Expected {elem} to be greater than {min}, got {obj[elem]}")
    
    if max is not None and obj[elem] > max:
        raise ValueError(f"Expected {elem} to be less than {max}, got {obj[elem]}")
