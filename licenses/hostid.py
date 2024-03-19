def ip_to_hostid(ip:str) -> str:
    spl = ip.split(".")
    hexstrings = [hex(int(x)) for x in spl]
    trimmed_hexstrings = [hexstring[2:] for hexstring in hexstrings]

    result = []
    for trimmed_hexstring in trimmed_hexstrings:
        if len(trimmed_hexstring) < 2:
            trimmed_hexstring = "0" + trimmed_hexstring
        result.append(trimmed_hexstring)
    return result[1] + result[0] + result[3] + result[2]

def hostid_to_ip(hostid:str) -> str:
    intlist = []
    intlist.append(int(hostid[2:4], 16))
    intlist.append(int(hostid[0:2], 16))
    intlist.append(int(hostid[6:8], 16))
    intlist.append(int(hostid[4:6], 16))
    strlist = [str(x) for x in intlist]
    return ".".join(strlist)

def main():
    # Example usage
    print(ip_to_hostid("192.168.11.22"))
    x = "a8c0160b"
    print(hostid_to_ip(x))


if __name__ == "__main__":
    main()