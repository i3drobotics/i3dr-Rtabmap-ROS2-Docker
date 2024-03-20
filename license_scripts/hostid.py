def ip_to_hostid(ip:str) -> str:
    """Converts IP address to host ID"""
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
    """Converts host ID to IP address"""
    intlist = []
    intlist.append(int(hostid[2:4], 16))
    intlist.append(int(hostid[0:2], 16))
    intlist.append(int(hostid[6:8], 16))
    intlist.append(int(hostid[4:6], 16))
    strlist = [str(x) for x in intlist]
    return ".".join(strlist)

def main():
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--id", "-i", type=str, required=True, help="Host ID")
    args = parser.parse_args()

    hostid = args.id
    print(hostid_to_ip(hostid))

    # Example usage
    # python hostid.py -i hostidfromlic

if __name__ == "__main__":
    main()