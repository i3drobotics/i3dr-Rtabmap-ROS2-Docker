from pathlib import Path
import re


def read_lic(lic_path:str|Path) -> tuple[str,str]:
    """Reads license file and returns host name and host ID"""
    with open(lic_path) as lic_file:
        lic_file_content = lic_file.read()

    regex = re.compile(r"LICENSESERVER \w+ \w+")
    match = regex.search(lic_file_content)

    host_name = ""
    host_id = ""
    if match:
        result = match.group().split(" ")
        host_name = result[1]
        host_id = result[2]
    return host_name, host_id


def main():
    script_dir = Path(__file__).parent.absolute()
    lic_path = script_dir / "2024_01_09_I3DRWL004_KinYip.lic"
    host_name, host_id = read_lic(lic_path)
    print(f"Host name = {host_name}\nHost ID = {host_id}")


if __name__ == "__main__":
    main()