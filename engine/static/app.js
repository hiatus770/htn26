const piece = {P:'♙︎',N:'♘︎',B:'♗︎',R:'♖︎',Q:'♕︎',K:'♔︎',p:'♟︎',n:'♞︎',b:'♝︎',r:'♜︎',q:'♛︎',k:'♚︎'};
let games = [], profiles = [], selected = null;
const $ = s => document.querySelector(s);

function drawBoard(element, fen) {
  element.replaceChildren(); const rows = fen.split(' ')[0].split('/');
  rows.forEach((row, rank) => { let file=0; for (const c of row) {
    const count = Number(c) || 1; for(let i=0;i<count;i++,file++) { const square=document.createElement('div'); square.className=`square ${(file+rank)%2?'dark':'light'}`;
      const pc=Number(c)?null:c; if(pc) {const span=document.createElement('span'); span.className=`piece ${pc===pc.toUpperCase()?'white':'black'}`;span.textContent=piece[pc]; square.append(span)} element.append(square); }
  }});
}
function lastMove(g){const m=g.moves.at(-1); return m ? `${m.side.toUpperCase()}: ${m.san}${m.is_capture?' × '+m.captured_piece:''}` : 'No completed moves';}
function render(){
  const holder=$('#games'); holder.replaceChildren(); $('#empty').hidden=games.length>0;
  if (!selected && games.length) selected=games[0].game_id;
  games.forEach(g=>{const node=$('#game-card').content.cloneNode(true); const card=node.querySelector('article'); card.classList.toggle('selected',g.game_id===selected); card.querySelector('h2').textContent=g.game_id;card.querySelector('.state').textContent=g.status.replaceAll('_',' '); drawBoard(card.querySelector('.board'),g.fen);card.querySelector('.profile').textContent=`${g.profile.display_name} — ${g.profile.attitude}`;card.querySelector('.last-move').textContent=lastMove(g);card.querySelector('.evaluation').textContent=`EVALUATION: ${(g.evaluation_label||'neutral').toUpperCase()}${g.evaluation_cp===null?'':` (${g.evaluation_cp} cp)`}`; const alert=card.querySelector('.alert');if(g.last_error){alert.hidden=false;alert.textContent=`DISCREPANCY: ${g.last_error}`}const inspect=card.querySelector('.inspect');inspect.textContent=g.game_id===selected?'Selected':'Inspect';inspect.onclick=()=>{selected=g.game_id;render();};holder.append(node)});
  if(selected) {const fresh=games.find(g=>g.game_id===selected);if(fresh) showDetail(selected,false);}
}
function showDetail(id, scroll=true){selected=id;const g=games.find(x=>x.game_id===id);if(!g)return;$('#detail').hidden=false;$('#detail-title').textContent=`BOARD ${g.game_id} / ROUND ${g.round_number}`;$('#detail-status').textContent=g.status.replaceAll('_',' ').toUpperCase();drawBoard($('#large-board'),g.fen);$('#facts').innerHTML=`<dt>Profile</dt><dd>${g.profile.display_name}</dd><dt>Attitude</dt><dd>${g.profile.attitude}</dd><dt>FEN</dt><dd>${g.fen}</dd><dt>Evaluation</dt><dd>${g.evaluation_label||'neutral'} ${g.evaluation_cp===null?'':`${g.evaluation_cp} cp`}</dd><dt>Pending robot move</dt><dd>${g.pending_san||'—'}</dd><dt>Result</dt><dd>${g.result||'active'} ${g.result_type?`(${g.result_type})`:''}</dd>`;$('#moves').replaceChildren(...g.moves.map(m=>{const li=document.createElement('li');li.textContent=`${m.side}: ${m.san}${m.is_capture?' × '+m.captured_piece:''}`;return li;}));$('#profile').replaceChildren(...profiles.map(p=>Object.assign(document.createElement('option'),{value:p.id,textContent:p.display_name,selected:p.id===g.profile_id})));if(scroll)$('#detail').scrollIntoView({behavior:'smooth'});}
async function refresh(){const response=await fetch('/v1/session');const data=await response.json();games=data.games||[];render();}
async function action(path, body){const r=await fetch(path,{method:'POST',headers:{'content-type':'application/json'},body:JSON.stringify(body)});const out=await r.json();$('#control-result').textContent=r.ok?'Action recorded.':(out.detail?.reason||out.detail||'Action failed.');await refresh();}
$('#forfeit-robot').onclick=()=>confirm('Record a robot forfeit?')&&action(`/v1/games/${selected}/forfeit`,{forfeiting_side:'robot'});
$('#forfeit-player').onclick=()=>confirm('Record a player forfeit?')&&action(`/v1/games/${selected}/forfeit`,{forfeiting_side:'player'});
$('#restart').onclick=()=>confirm('Archive this round and start a new one?')&&action(`/v1/games/${selected}/restart`,{profile_id:$('#profile').value});
async function boot(){profiles=await (await fetch('/v1/profiles')).json();await refresh();const ws=new WebSocket(`${location.protocol==='https:'?'wss':'ws'}://${location.host}/v1/events`);ws.onopen=()=>{$('#connection').textContent='LIVE';$('#connection').className='online';ws.send('ready')};ws.onmessage=()=>refresh();ws.onclose=()=>{$('#connection').textContent='RECONNECTING';$('#connection').className='';setTimeout(boot,2000)}}boot();
