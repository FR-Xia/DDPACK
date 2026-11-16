function report = validate_paired_evidence()
%VALIDATE_PAIRED_EVIDENCE Reject paired figures/tables bound to stale sources.
root=fileparts(fileparts(fileparts(mfilename('fullpath'))));
manifest=readtable(fullfile(root,'experiments','results','source_sha256.csv'), ...
    'TextType','string');
assert(height(manifest)>0,'Missing paired source provenance.');
for k=1:height(manifest)
    filename=fullfile(root,manifest.file(k));
    fid=fopen(filename,'rb'); assert(fid>=0,'Missing paired source: %s',filename);
    bytes=fread(fid,Inf,'*uint8'); fclose(fid);
    digest=java.security.MessageDigest.getInstance('SHA-256'); digest.update(bytes);
    actual=lower(reshape(dec2hex(typecast(digest.digest(),'uint8'),2).',1,[]));
    assert(strcmp(actual,manifest.sha256(k)), ...
        'Paired evidence is stale: %s. Re-run the paired experiment/analysis.',manifest.file(k));
end
report=struct('success',true,'source_files_checked',height(manifest));
end
